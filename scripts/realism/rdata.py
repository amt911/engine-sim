"""Where the realism gates keep their data: the outside recordings' analysis caches, the renders and
the per-build tables. Never in the repository (the recordings are private and large): the folder named
by $REALISM_DATA, by default target/scratch-refs/realism-agent of this checkout's main repository."""
import os

_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def _default():
    # a worktree under target/ shares the main checkout's data
    p = _REPO
    while p != os.path.dirname(p):
        cand = os.path.join(p, "target", "scratch-refs", "realism-agent")
        if os.path.isdir(cand):
            return cand
        p = os.path.dirname(p)
    return os.path.join(_REPO, "target", "scratch-refs", "realism-agent")


DATA = os.environ.get("REALISM_DATA") or _default()
