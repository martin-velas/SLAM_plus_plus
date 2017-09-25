To build the documentation, you will need:

* doxygen
* dot for diagrams (part of the graphviz package; see
  http://graphviz.org/Download.php)
* tex for equations

You need to edit Doxyfile and change the path to your dot,
otherwise you won't have the diagrams generated. Look for
a line which starts with DOT_PATH.

To generate the documentation, cd over to the same directory
as the Doxyfile and do:

> doxygen Doxyfile

This generates a pretty long log, and writes the outputs
in the ../build/html directory. You can change the output
path by changing the OUTPUT_DIRECTORY variable in the Doxyfile.


Troubleshooting

Doxygen does not make it very easy to work with relative paths,
so if you run into trouble, it may be that some absolute path
on the developer machine was forgotten in the Doxyfile. Look
for Windows paths such as "Program Files" or "my-projects" and
replace them as needed.
