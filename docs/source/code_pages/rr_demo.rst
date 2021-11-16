=======
rr_demo
=======

This package is useful to review if you are new to documentation.
Here is an ever-evolving directory full of useful tips for documenting 
your code. Just like any other part of the RoboRacing repository, if you feel
like you can contribute, please submit a pull request and add you advice!

As you develop a package, you will want to document it. We use these Sphinx websites to 
display documentation. The main page for the package can have some important files useful
throughout all nodes inside the package.

For example, imagine **Driver.cpp** is used in all nodes in the rr_demo package. We could
include it by writing this code:

.. code-block::

    .. doxygenfile:: driver.cpp
        :project: rr_demo

Which would appear as:

.. doxygenfile:: driver.cpp
    :project: rr_demo
