"""Preserve all samples in headerless EyeLink sample-only ASCII exports."""

import pytest

from tabletop_py.gaze.edf import asc_to_df


@pytest.mark.parametrize("binocular", [False, True])
@pytest.mark.parametrize("count", [1, 3])
def test_sample_only_ascii_preserves_first_sample(tmp_path, binocular, count):
    path = tmp_path / "samples.asc"
    rows = []
    for index in range(count):
        values = [1000 + index, 10, 20, 30]
        if binocular:
            values.extend([40, 50, 60])
        values.append(95 if index == 0 else 127)
        rows.append("\t".join(map(str, values)))
    path.write_text("\n".join(rows) + "\n")

    result = asc_to_df(str(path))

    assert len(result) == count
    assert result["time"].tolist() == [index / 1000 for index in range(count)]
    assert result["input"].tolist() == [95] + [127] * (count - 1)
    assert result["left_x"].iloc[0] == 10
    if binocular:
        assert result["right_x"].iloc[0] == 40
