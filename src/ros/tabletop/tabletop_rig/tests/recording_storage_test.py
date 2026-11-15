from copy import deepcopy
from pathlib import Path

import pytest
from tabletop_rig.utils import recording_storage as storage

SOURCE = "//169.254.101.0/tabletop"


@pytest.fixture
def nas(tmp_path, monkeypatch):
    mount = tmp_path / "nas"
    directory = mount / "sessions"
    directory.mkdir(parents=True)
    info = {
        "target": str(mount),
        "source": SOURCE,
        "fstype": "cifs",
        "options": "rw,cache=none,mfsymlinks,nosuid,nodev,noexec",
    }
    monkeypatch.setattr(storage, "mount_info", lambda *a, **kw: deepcopy(info))
    return mount, directory, info


def test_nas_mount_and_container_subdirectory_source(nas):
    mount, directory, info = nas
    info["source"] += "[/sessions]"
    assert (
        storage.validate_nas_storage(str(directory), str(mount), SOURCE)[
            "available_bytes"
        ]
        > 0
    )


@pytest.mark.parametrize(
    "field,value",
    [
        ("fstype", "ext4"),
        ("source", "//wrong/share"),
        ("options", "rw,cache=strict,mfsymlinks"),
        ("options", "rw,cache=none"),
        ("options", "ro,cache=none,mfsymlinks"),
    ],
)
def test_nas_rejects_wrong_or_unsafe_mount(nas, field, value):
    mount, directory, info = nas
    info[field] = value
    with pytest.raises(RuntimeError):
        storage.validate_nas_storage(str(directory), str(mount), SOURCE)


def test_nas_rejects_escaped_directory(nas, tmp_path):
    mount, directory, _ = nas
    with pytest.raises(RuntimeError, match="outside"):
        storage.validate_nas_storage(str(tmp_path), str(mount), SOURCE)


def test_local_backend_unchanged(monkeypatch):
    monkeypatch.delenv("TABLETOP_RECORDING_BACKEND", raising=False)
    monkeypatch.setattr(
        storage,
        "mount_info",
        lambda *a, **kw: pytest.fail("local must not invoke NAS checks"),
    )
    assert (
        storage.validate_recording_environment("/missing/local/path") is None
    )


def test_invalid_backend_is_not_a_silent_local_fallback(monkeypatch):
    monkeypatch.setenv("TABLETOP_RECORDING_BACKEND", "typo")
    with pytest.raises(RuntimeError, match="local or nas"):
        storage.validate_recording_environment("/tmp")


def test_nas_requires_explicit_expected_share(nas):
    mount, directory, _ = nas
    with pytest.raises(RuntimeError, match="requires"):
        storage.validate_nas_storage(str(directory), str(mount), "")


def test_reader_profile_is_bounded_best_effort_and_does_not_mutate_input():
    original = {
        "/cam_sync/camera/image_raw/compressed": {
            "depth": 32,
            "reliability": "best_effort",
        },
        "/joint_states": {"depth": 500, "reliability": "best_effort"},
    }
    expected = deepcopy(original)
    result = storage.recorder_qos_profiles(original, nas=True)
    assert original == expected
    assert result["/cam_sync/camera/image_raw/compressed"] == {
        "history": "keep_last",
        "depth": 512,
        "reliability": "best_effort",
    }
    assert result["/joint_states"] == original["/joint_states"]
    assert storage.recorder_qos_profiles(original, nas=False) == original


def test_cifs_counters_select_only_requested_share_and_sum_connections():
    text = r"""1) \\169.254.101.0\tabletop
Bytes read: 123 Bytes written: 999
2) \\elsewhere\other
Bytes read: 99999 Bytes written: 0
3) \\169.254.101.0\tabletop
Bytes read: 456 Bytes written: 999
"""
    assert storage.cifs_read_bytes(text, SOURCE) == 579
    with pytest.raises(RuntimeError):
        storage.cifs_read_bytes(text, "//missing/share")


@pytest.mark.parametrize(
    "counts,failed",
    [([10, 10], False), ([0, 100_000_000], True), ([100, 0], True)],
)
def test_nas_read_guard_and_counter_reset(nas, monkeypatch, counts, failed):
    mount, directory, _ = nas
    counter = iter(counts)
    monkeypatch.setattr(storage, "cifs_read_bytes", lambda *a: next(counter))
    monkeypatch.setattr(Path, "read_text", lambda *a, **kw: "")
    clock = iter([10.0, 11.0])
    monkeypatch.setattr(storage.time, "monotonic", lambda: next(clock))
    monkeypatch.setattr(storage.time, "sleep", lambda _: None)
    if failed:
        with pytest.raises(RuntimeError):
            storage.check_nas_idle(
                str(directory), str(mount), SOURCE, seconds=1
            )
    else:
        assert (
            storage.check_nas_idle(
                str(directory), str(mount), SOURCE, seconds=1
            )["read_mib_s"]
            == 0
        )
