# Local Release Candidate Checklist

Prepared pair: Unity package 0.3.0, QuadSimLib 0.16.0, protocol 1.0.

- [ ] SDK focused tests pass on Python 3.10 through 3.14.
- [ ] Wheel and source distribution pass `twine check` and fresh-install smoke.
- [ ] Unity EditMode and PlayMode tests pass on 6000.5.8f1.
- [ ] Linux and Windows players build.
- [ ] Linux built-player RPC smoke emits a valid provenance manifest.
- [ ] Unity logs pass the exception/protocol/drop gate.
- [ ] Dependency and secret scans pass.
- [ ] Compatibility table and changelogs match the artifacts.
- [ ] Release commit hashes are recorded and both worktrees are reviewed.
- [ ] Create immutable tags only after all local and CI checks pass.
- [ ] Publish the GitHub/UPM release before documenting its tag as available.
- [ ] Upload the already-tested SDK artifacts to TestPyPI or PyPI only after
      explicit approval; do not rebuild between validation and upload.

Unity license credentials and PyPI tokens belong in the CI secret store and
must never be committed or copied into logs.
