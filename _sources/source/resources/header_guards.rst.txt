============================
What even are header guards?
============================

Okay I have assembled two answers and learned something along the way. First I will show the naive reason. I have been trying to learn Vim so let me know if it is unreadable and I can recreate it.

.. image:: /_static/header_guard_1.png
   :alt: header_guard_1
   :class: with-shadow

What we see here is that by including “header.h” twice, the add function is being defined twice. Adding a header guard will solve this.

.. image:: /_static/header_guard_2.png
   :alt: header_guard_2
   :class: with-shadow

A more common use case is that when files are linked together they have duplicate “includes” which causes this same issue as above of “redefining”.

.. image:: /_static/header_guard_3.png
   :alt: header_guard_3
   :class: with-shadow

Here I have two headers. I created a typedef called my_type which is just an int. I then wanted to use that typedef in both header.h and in code.c. Interestingly, it is okay to re-declare the same typedef or same method header multiple times; so I decided to add the subtract function. Now subtraction is included in both header.h and code.c. The linker does not allow the two function implementations of the same name to be included twice! To let the compiler know that the subtraction function is actually the same function in both cases, we can use a header guard.

.. image:: /_static/header_guard_4.png
   :alt: header_guard_4
   :class: with-shadow

In a larger example you would want to not have implementation in a header file but in another file but that would have increased the size of this project which I did not want to do. You are allowed to redefine the same method header as many times as you want, as long as it is identical. But when you start to add implementation, that is when the linker throws a fit.

You can see that the code compiles and works without a header guard on header.h but you should always add the header guard in case you change the file in the future and it becomes necessary.

As an aside, #pragma once is not identical to the more traditional header guard:

```
#ifndef HEADER_H
#define HEADER_H
//code
#endif
```

#pragma once will be good enough for our purposes but I have heard that it is not supported in all C implementations. You can read this header guard and see that is is just an if statement staying “if not defined HEADER_H, then define HEADER_H and run this code”.