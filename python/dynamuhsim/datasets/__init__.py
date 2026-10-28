"""Shared, hash-bound normalization math for closed episode data."""
import hashlib
from pathlib import Path

PINS = {'normalization.py': '1bee12f29d587a09e08d0a44388bc8767131b90e0bee7ddb101e9a03b347ef3e', 'normalization_contract.json': 'b3efb34a9efa0145071a40498f5309493fb472d87538a1207ce799869f5329f6'}

def verify_math():
    for name, expected in PINS.items():
        path = Path(__file__).parent / name
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError("Frozen normalizer math changed: " + name)

verify_math()
