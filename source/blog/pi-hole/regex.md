---
title: Using Regex to Block Sites
date: 5 Jan 2020
---

I kept getting sent to the crap "You Wone a Prize" website and tried to block it,
but the name kept changing. So I was able to use `pihole`'s regex command to identify
the pattern and block it.

```regex
/^mobile(.*)noname(.*)live$/gi
```

- General syntax: `/<matching filters>/gi`
- `^mobile`: identifies the site will always start with *mobile*
- `(.*)`: followed by 0 or more characters ... `.` is a wildcard that matches everything
- `noname`: is an exact string match
- `(.*)`: again, 0 or more characters
- `live$`: the site's name will end in *live*
- `/gi`: `g`(global) means everything has to match (like an AND) and `i` is case insensitive

The references below were really helpful!

# References

- Interactive regex [site](https://regex101.com/r/cO8lqs/2)
- Regex [tutorial](https://medium.com/factory-mind/regex-tutorial-a-simple-cheatsheet-by-examples-649dc1c3f285)
- Regex [cookbook](https://medium.com/factory-mind/regex-cookbook-most-wanted-regex-aa721558c3c1?)
