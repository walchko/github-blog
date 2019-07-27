---
date: 14 Mar 2019
---

![](pics/cpp.png){width=200px}

# Multiprocessing

You can create multiple processes using the `fork` command. Basically the program is
run multiple times and if:

- `pid` = 0: this is the child process
- `pid` > 0: this is the parent process and the `pid` is for a child process

<script src="https://gist.github.com/walchko/72a0c837e8de32bd0173097bbea97395.js"></script>

# References

- [A great overview](static/multiprocessing.pdf)
- [another reg](https://www.usna.edu/Users/cs/aviv/classes/ic221/s16/lec/14/lec.html)
- [ref](https://ece.uwaterloo.ca/~dwharder/icsrts/Tutorials/fork_exec/)
- [ref](http://timmurphy.org/2014/04/26/using-fork-in-cc-a-minimum-working-example/)
- [ref](http://thispointer.com/creating-a-new-process-using-fork-tutorial-example/)
