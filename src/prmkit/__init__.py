"""Independent planning kit. No imports from the research repository."""
__version__ = "0.4.0"


def run_case(*args, **kwargs):
    """Run a configurable research study; see prmkit.cases.run_case."""
    from .cases import run_case as execute
    return execute(*args, **kwargs)


def list_cases():
    """Return the supported case IDs and descriptions."""
    from .cases import list_cases as catalog
    return catalog()

