Here are conversion utilities from the formats used by bundle
adjustment (BA) tools to graph files that SLAM++ can process.
This can be used e.g. for benchmarking.

Currently there is bundler2graph (for Bundler .out files) and
nvm2graph (for VisualSFM .nvm files).

These tools are provided "as is" and may not always work with
the latest version of the third party tools outputs. We won't
be implementing convertors from SLAM++ to .out or .nvm unless
there is a very good reason for it.
