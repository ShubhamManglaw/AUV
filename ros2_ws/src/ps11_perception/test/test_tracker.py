"""Unit tests for ObjectTracker (T2.6 / Q1).

Acceptance checks:
1. Fewer than min_hits (5 frames) gives no confirmed tracks.
2. One object tracked over 30 frames produces exactly 1 consistent track ID with hits increasing to 30.
3. Majority-vote class selection over track history.
4. Mean confidence filtering (mean confidence < 0.4 not confirmed).
"""

import numpy as np
from ps11_perception.tracker import ObjectTracker


def test_fewer_than_min_hits_gives_none() -> None:
    """Tracks with fewer than min_hits (5) detections are not published."""
    tracker = ObjectTracker(min_hits=5, min_mean_conf=0.4)

    box = np.array([[100.0, 100.0, 150.0, 150.0]])
    conf = np.array([0.8])
    cls_id = np.array([1])

    # Frames 1 through 4: should return 0 confirmed tracks
    for frame_idx in range(1, 5):
        confirmed = tracker.update(box, conf, cls_id)
        assert len(confirmed) == 0, (
            f"Expected 0 tracks at frame {frame_idx}, got {len(confirmed)}"
        )

    # Frame 5: reaches min_hits=5, should now be confirmed
    confirmed = tracker.update(box, conf, cls_id)
    assert len(confirmed) == 1, "Expected 1 confirmed track at frame 5"
    assert confirmed[0].hits == 5
    assert confirmed[0].class_id == 1
    assert confirmed[0].u == 125.0
    assert confirmed[0].v == 125.0
    assert confirmed[0].width == 50.0
    assert confirmed[0].height == 50.0


def test_one_object_over_30_frames_gives_1_track_id() -> None:
    """One object tracked across 30 consecutive frames produces exactly 1 track ID."""
    tracker = ObjectTracker(min_hits=5, min_mean_conf=0.4)

    # Move box slightly to simulate realistic motion
    track_ids_seen = set()
    hits_progression = []

    for i in range(30):
        # Shift 1 px each frame
        box = np.array([[100.0 + i, 100.0 + i, 150.0 + i, 150.0 + i]])
        conf = np.array([0.85])
        cls_id = np.array([2])

        confirmed = tracker.update(box, conf, cls_id)
        if len(confirmed) > 0:
            track_ids_seen.add(confirmed[0].track_id)
            hits_progression.append(confirmed[0].hits)

    # Exactly 1 track ID throughout
    assert len(track_ids_seen) == 1, f"Expected 1 unique track ID, got {track_ids_seen}"
    # Confirmed from frame 5 through 30 (26 frames confirmed)
    assert len(hits_progression) == 26
    assert hits_progression[0] == 5
    assert hits_progression[-1] == 30


def test_majority_vote_class() -> None:
    """Class assignment reflects the majority vote across detection history."""
    tracker = ObjectTracker(min_hits=5, min_mean_conf=0.4)

    box = np.array([[200.0, 200.0, 260.0, 260.0]])
    conf = np.array([0.9])

    # 4 frames detected as class 0
    for _ in range(4):
        tracker.update(box, conf, np.array([0]))

    # Next 6 frames detected as class 1 (6 vs 4)
    confirmed_last = None
    for _ in range(6):
        res = tracker.update(box, conf, np.array([1]))
        if res:
            confirmed_last = res[0]

    assert confirmed_last is not None
    assert confirmed_last.hits == 10
    # Majority vote must be 1 (6 votes vs 4 votes)
    assert confirmed_last.class_id == 1


def test_low_confidence_filtered() -> None:
    """Tracks with mean confidence below min_mean_conf are not confirmed."""
    tracker = ObjectTracker(min_hits=5, min_mean_conf=0.4)

    box = np.array([[50.0, 50.0, 80.0, 80.0]])
    conf = np.array([0.2])  # 0.2 < 0.4
    cls_id = np.array([0])

    for _ in range(10):
        confirmed = tracker.update(box, conf, cls_id)
        assert len(confirmed) == 0, "Low-confidence detections should not be confirmed"
