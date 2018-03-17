![](pics/kicad.png)

# KiCad

## Adding Libraries

1. Find the library you want on [KiCad Libraries](https://kicad.github.io/)
2. Download it
3. Add the path to the schematics editor

## Tutorials

- [Sparkfun's KiCad Overview](https://www.sparkfun.com/news/2041)
- [Sparkfun's library setup and how to create a custom footprint library](https://learn.sparkfun.com/tutorials/beginners-guide-to-kicad/creating-a-custom-kicad-footprint-library)
    - If you ever mess up your libraries, delete `fp-lib-table` and restart kicad.
    Kicad will setup the libraries back to the default values.
        - Windows 10: `C:\Users\Nathan\AppData\Roaming\kicad\fp-lib-table`
        - macOS: it is in the same folder as the project (\*.pro)

# Gerber Viewer

Plot your PCB designs and zip up the files. Then drag/drop them on to: https://gerber-viewer.easyeda.com/
