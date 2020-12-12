---
title: GeoJSON
date: 19 Nov 2019
image: "https://i.pinimg.com/564x/a3/64/c6/a364c662c634560c18ef633501b7f0e3.jpg"
image-height: "400px"
---

GeoJSON is an open standard format designed for representing simple geographical features, along with
their non-spatial attributes. It is based on the JavaScript Object Notation (JSON) and capabile of
handling points, linestrings, polygons, and more.

## Example

```json
{
  "type": "FeatureCollection",
  "features": [
    {
      "type": "Feature",
      "geometry": {
        "type": "Point",
        "coordinates": [102.0, 0.5]
      },
      "properties": {
        "prop0": "value0"
      }
    },
    {
      "type": "Feature",
      "geometry": {
        "type": "LineString",
        "coordinates": [
          [102.0, 0.0], [103.0, 1.0], [104.0, 0.0], [105.0, 1.0]
        ]
      },
      "properties": {
        "prop0": "value0",
        "prop1": 0.0
      }
    },
    {
      "type": "Feature",
      "geometry": {
        "type": "Polygon",
        "coordinates": [
          [
            [100.0, 0.0], [101.0, 0.0], [101.0, 1.0],
            [100.0, 1.0], [100.0, 0.0]
          ]
        ]
      },
      "properties": {
        "prop0": "value0",
        "prop1": { "this": "that" }
      }
    }
  ]
}
```

# References

- [Wikipedia: GeoJSON](https://en.wikipedia.org/wiki/GeoJSON)
- [Live GeoJSON Editor](http://geojson.io)
- [Simple summary of geojson](https://medium.com/@sumit.arora/what-is-geojson-geojson-basics-visualize-geojson-open-geojson-using-qgis-open-geojson-3432039e336d)
