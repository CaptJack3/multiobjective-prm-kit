# Add the project video

The root README already reserves its opening visual block for the project video.
Until the final video is ready, it shows a real kitchen example linked to the
research-example guide. There is no broken placeholder URL.

1. Upload the final video to its chosen host, or attach it to a GitHub release.
2. Add a representative poster image under `docs/media/` (or reuse an existing preview).
3. Replace the image/link directly below `PROJECT_VIDEO_START` in `README.md`:

```markdown
[![Watch the PRM Kit project video](docs/media/project-video.jpg)](VIDEO_URL)
```

Use the actual URL in place of `VIDEO_URL`. Keep a one-line caption describing
what the viewer will see. A linked thumbnail works without requiring GitHub's
Markdown renderer to play an arbitrary relative MP4 embed. Keep the quickstart
immediately below the visual block so newcomers can run the code after watching.

The current research videos are kept with their scene data. Their recordings
show different historical hardware revisions, so preserve the gallery's labels
when selecting clips for the project video.
