# Present the project

The README opens with the project title, navigation, the uploaded project video
and a three-scene image gallery. The installation instructions follow this visual
introduction. Click a gallery image to inspect it at full size.

## Update the video

1. Upload the replacement video using GitHub's Markdown attachment uploader.
2. Copy its `https://github.com/user-attachments/assets/...` URL.
3. Replace the standalone URL between `PROJECT_VIDEO_START` and
   `PROJECT_VIDEO_END` in the root `README.md`.

Keep the URL on its own line, with blank lines around it, outside HTML blocks
and code fences. This preserves GitHub's attachment rendering. The video is
hosted on GitHub; it is not bundled in a source ZIP or an offline clone.

If the video moves to a different host, use a linked poster image instead:

```markdown
[![Watch the PRM Kit project video](docs/media/project-video.jpg)](VIDEO_URL)
```

Add the real poster file and replace `VIDEO_URL` before committing that change.

## Update the gallery

The three README images link to the original multiview previews for the kitchen,
workbench and shelf scenes. Keep their labels and sizes consistent, and place
the gallery before the quickstart. Reuse the actual research assets.

The archived scene previews and later route videos show different historical
hardware revisions. Preserve the revision notes in the
[research-example guide](RESEARCH_EXAMPLES.md) when changing the presentation.
