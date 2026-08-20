# Vendored SCLIP Runtime

Source: https://github.com/wangf3014/SCLIP

Pinned commit: `3608360267b6130c1ef18090d7289f17c771cb90`

Only the runtime `clip/` package, upstream README, and MIT license are vendored.
The segmentation evaluation framework and dataset configs are not required by
YOPO-Rally. CLIP weights are downloaded by the upstream loader on first use.
