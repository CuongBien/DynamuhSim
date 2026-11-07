"""Errors raised at the data and configuration boundaries."""


class ContractError(ValueError):
    """A value does not satisfy a NarrowNav data contract."""


class ConfigurationError(ContractError):
    """A composed configuration is incomplete or inconsistent."""
