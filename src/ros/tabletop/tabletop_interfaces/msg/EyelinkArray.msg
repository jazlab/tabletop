# Batch of 50 consecutive Eyelink gaze samples, published in batched mode to reduce message overhead.
# Published by: eyelink node (nodes/eyelink.py, topic ~/sample_array) when publish_batched=true (default).
# Consumed by: subscribers to the eyelink/sample_array topic (e.g., for recording or real-time processing).

tabletop_interfaces/Eyelink[50] samples  # Fixed-length array of 50 Eyelink samples; filled sequentially, published when the 50th slot is filled
