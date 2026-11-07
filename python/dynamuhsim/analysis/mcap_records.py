"""Read-only MCAP record traversal in physical write order, including both clocks.

Layout follows the installed mcap_vendor/mcap/types.hpp. Zstd uses the already
installed system library; no Python package or network installation is needed.
"""
import ctypes
import ctypes.util
from pathlib import Path
import struct
import zlib

MAGIC = b'\x89MCAP0\r\n'


class McapError(ValueError):
    pass


class Cursor:
    def __init__(self, data):
        self.data, self.pos = data, 0

    def take(self, n):
        if self.pos + n > len(self.data):
            raise McapError(f'Truncated record field at payload byte {self.pos}, requested {n}')
        value = self.data[self.pos:self.pos+n];self.pos += n
        return value

    def number(self, fmt):
        return struct.unpack('<'+fmt, self.take(struct.calcsize('<'+fmt)))[0]

    def string(self):
        return self.take(self.number('I')).decode('utf-8')


def records(data, origin):
    pos = 0
    while pos < len(data):
        if len(data)-pos < 9:
            raise McapError(f'{origin}: truncated record header at {pos}')
        opcode, length = struct.unpack_from('<BQ', data, pos)
        if pos+9+length > len(data):
            raise McapError(f'{origin}: opcode {opcode} truncated at {pos}, length {length}')
        yield opcode, data[pos+9:pos+9+length], pos
        pos += 9+length


def decompress(compression, payload, size):
    if compression == '':
        result = payload
    elif compression == 'zstd':
        library = ctypes.CDLL(ctypes.util.find_library('zstd'))
        library.ZSTD_decompress.argtypes = [ctypes.c_void_p,ctypes.c_size_t,ctypes.c_void_p,ctypes.c_size_t]
        library.ZSTD_decompress.restype = ctypes.c_size_t
        library.ZSTD_isError.argtypes = [ctypes.c_size_t];library.ZSTD_isError.restype = ctypes.c_uint
        target = ctypes.create_string_buffer(size);source = ctypes.create_string_buffer(payload)
        length = library.ZSTD_decompress(target,size,source,len(payload))
        if library.ZSTD_isError(length):raise McapError(f'Zstd decompression failed: {length}')
        result = target.raw[:length]
    elif compression == 'lz4':
        import lz4.frame
        result = lz4.frame.decompress(payload)
    else:
        raise McapError('Unsupported compression: '+compression)
    if len(result) != size:raise McapError(f'Chunk size mismatch: {len(result)} != {size}')
    return result


class McapFile:
    def __init__(self, path):
        self.path = Path(path)
        self.schemas, self.channels = {}, {}
        self.schema_locations = {}
        self.integrity = dict(chunks=0, chunk_crc_verified=0, chunk_crc_disabled=0,
                              summary_crc='NOT_MEASURED', data_crc='NOT_MEASURED', footer=False)

    def messages(self):
        data = self.path.read_bytes()
        if data[:8] != MAGIC or data[-8:] != MAGIC:raise McapError(f'{self.path}: invalid opening/trailing MCAP magic')
        for opcode, payload, relative in records(data[8:-8], str(self.path)):
            offset = relative+8
            if opcode == 6:
                c=Cursor(payload);start=c.number('Q');end=c.number('Q');size=c.number('Q');crc=c.number('I')
                compression=c.string();packed=c.take(c.number('Q'))
                if c.pos!=len(payload):raise McapError(f'{offset}: extra chunk bytes')
                try:unpacked=decompress(compression,packed,size)
                except McapError as error:raise McapError(f'{self.path}, chunk byte {offset}: {error}') from error
                self.integrity['chunks']+=1
                if crc:
                    if zlib.crc32(unpacked)!=crc:raise McapError(f'{self.path}, chunk byte {offset}: chunk CRC mismatch')
                    self.integrity['chunk_crc_verified']+=1
                else:self.integrity['chunk_crc_disabled']+=1
                for code,body,inner in records(unpacked,f'file byte {offset}, uncompressed chunk'):
                    row=self._record(code,body,dict(file=str(self.path),chunk_offset=offset,uncompressed_offset=inner))
                    if row is not None:yield row
            elif opcode == 2:
                start,summary_offsets,crc=struct.unpack('<QQI',payload)
                if offset+9+len(payload)!=len(data)-8:raise McapError('Footer is not the final record')
                if start and not 8<=start<=offset:raise McapError('Invalid summary start')
                if crc and zlib.crc32(data[start:offset+25])!=crc:raise McapError('Summary CRC mismatch')
                self.integrity.update(footer=True,summary_crc='VERIFIED' if crc else 'NOT_MEASURED')
            elif opcode == 15:
                crc=struct.unpack('<I',payload)[0]
                if crc and zlib.crc32(data[:offset])!=crc:raise McapError('Data section CRC mismatch')
                self.integrity['data_crc']='VERIFIED' if crc else 'NOT_MEASURED'
            else:
                row=self._record(opcode,payload,dict(file=str(self.path),file_offset=offset))
                if row is not None:yield row
        if not self.integrity['footer']:raise McapError('Missing footer')

    def _record(self, opcode, payload, location):
        c=Cursor(payload)
        if opcode == 3:
            sid=c.number('H');name=c.string();encoding=c.string();definition=c.take(c.number('I')).decode('utf-8')
            schema=dict(name=name,encoding=encoding,definition=definition)
            if sid in self.schemas and self.schemas[sid]!=schema:raise McapError(f'Schema {sid} redefined')
            self.schemas[sid]=schema
            self.schema_locations.setdefault(sid,location)
        elif opcode == 4:
            cid=c.number('H');sid=c.number('H');topic=c.string();encoding=c.string();size=c.number('I')
            metadata=Cursor(c.take(size));values={}
            while metadata.pos<size:
                key=metadata.string();values[key]=metadata.string()
            channel=dict(topic=topic,schema_id=sid,message_encoding=encoding,metadata=values)
            if cid in self.channels and self.channels[cid]!=channel:raise McapError(f'Channel {cid} redefined')
            self.channels[cid]=channel
        elif opcode == 5:
            cid=c.number('H');sequence=c.number('I');log=c.number('Q');publish=c.number('Q')
            if cid not in self.channels:raise McapError(f'Unknown channel {cid} at {location}')
            channel=self.channels[cid]
            if channel['schema_id'] not in self.schemas:raise McapError(f'Unknown schema at {location}')
            return dict(channel=cid,schema_id=channel['schema_id'],sequence=sequence,record_ns=log,publish_ns=publish,
                        topic=channel['topic'],schema=self.schemas[channel['schema_id']],
                        payload=payload[c.pos:],location=location)
        return None
