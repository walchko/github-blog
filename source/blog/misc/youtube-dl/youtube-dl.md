---
title: Youtube-dl
date: 11 Jan 2021
---

```
youtube-dl -F 'http://www.youtube.com/watch?v=P9pzm5b6FFY'

[youtube] Setting language
[youtube] P9pzm5b6FFY: Downloading webpage
[youtube] P9pzm5b6FFY: Downloading video info webpage
[youtube] P9pzm5b6FFY: Extracting video information
[info] Available formats for P9pzm5b6FFY:
format code extension resolution  note 
140         m4a       audio only  DASH audio , audio@128k (worst)
160         mp4       144p        DASH video , video only
133         mp4       240p        DASH video , video only
134         mp4       360p        DASH video , video only
135         mp4       480p        DASH video , video only
136         mp4       720p        DASH video , video only
17          3gp       176x144     
36          3gp       320x240     
5           flv       400x240     
43          webm      640x360     
18          mp4       640x360     
22          mp4       1280x720    (best)
```

Best quality is 22, you can download it with:

- `youtube-dl -f 22 'http://www.youtube.com/watch?v=P9pzm5b6FFY'`
- `youtube-dl -f best 'http://www.youtube.com/watch?v=P9pzm5b6FFY'`
- `youtube-dl -f 'bestvideo[ext=mp4]+bestaudio[ext=m4a]/bestvideo+bestaudio' --merge-output-format mp4 'http://www.youtube.com/watch?v=P9pzm5b6FFY'
`
- `youtube-dl -f best 'http://www.youtube.com/watch?v=P9pzm5b6FFY'`

## Reference

- askubuntu: [How to select video quality from youtube-dl?](https://askubuntu.com/questions/486297/how-to-select-video-quality-from-youtube-dl)
