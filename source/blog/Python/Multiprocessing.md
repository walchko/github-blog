<img src="pics/python-snake.jpg" width="100%">

# Multi-Processing

I generally dislike how python does threads, so I don't use them. They tend
to be more trouble than they are worth.

## Sharing Namespace

Using the `SyncManager`, you can provide a shared namespace between running
processes. Data in the namespace is automagically updated when a new assignment
is made.

- Updates namespace: namespace.data = new_data
- Doesn't update namespace: namespace.data.append(new value)



<script src="https://gist.github.com/walchko/22c79428d11fcdc188dd7b934dce968a.js"></script>
