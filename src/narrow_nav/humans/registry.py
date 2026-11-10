"""Factory for installed human navigation backends."""

from narrow_nav.core.errors import ConfigurationError
from narrow_nav.core.protocols import RegisteredHumanBackend

from .backends.jupedsim_avm import AvmHumanBackend
from .backends.jupedsim_csm import CsmHumanBackend
from .backends.jupedsim_sfm import SfmHumanBackend
from .backends.orca import OrcaHumanBackend
from .scripted import ScriptedHumanBackend

_BACKENDS = {
    ScriptedHumanBackend.backend_id: ScriptedHumanBackend,
    OrcaHumanBackend.backend_id: OrcaHumanBackend,
    SfmHumanBackend.backend_id: SfmHumanBackend,
    AvmHumanBackend.backend_id: AvmHumanBackend,
    CsmHumanBackend.backend_id: CsmHumanBackend,
}


def create_human_backend(backend_id: str) -> RegisteredHumanBackend:
    try:
        factory = _BACKENDS[backend_id]
    except KeyError:
        raise ConfigurationError(f"unsupported human backend {backend_id!r}") from None
    return factory()
