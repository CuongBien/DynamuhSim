"""Filesystem guard for experimental model exports; never replace frozen/default artifacts."""
from pathlib import Path


def new_artifact_dir(target, classifier_dir):
    target = Path(target).resolve()
    protected = [Path(classifier_dir).resolve()/name for name in ['artifacts', 'artifacts_legacy']]
    if any(target == p or target.is_relative_to(p) for p in protected):
        raise ValueError('Cannot overwrite frozen default or legacy artifacts')
    target.mkdir(parents=True, exist_ok=False)
    return target
