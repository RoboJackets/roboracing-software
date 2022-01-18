# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'Robojackets Roboracing'
copyright = '2021, Robojackets'
author = 'Charles Jenkins'

# The full version, including alpha/beta/rc tags
release = '1.0'


# -- General configuration ---------------------------------------------------
# The name for this set of Sphinx documents.  If None, it defaults to
# "<project> v<release> documentation".
html_title = project

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [ 'sphinxcontrib.plantuml', 'breathe' ]

breathe_default_project = 'rr_description'

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

html_favicon = '_static/favicon.png'
html_logo = '_static/robobuzz.png'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

breathe_projects = {
    'rr_description': 'xml',
    'rr_demo': 'xml'
}

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'furo'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

import os
home = os.getenv("HOME")
plantuml = f'java -jar {home}/java/plantuml.jar'

html_context = {
    'web_base_url' : 'https://github.com/robojackets/roboracing-software/edit/ros2/main/docs',
    'vcs_host' : 'Github'
}

html_sidebars = {
    "**" : [  "sidebar/brand.html",
  "sidebar/search.html",
  "sidebar/scroll-start.html",
  "sidebar/navigation.html",
  "sidebar/ethical-ads.html",
  "sidebar/gitadd.html",
  "sidebar/scroll-end.html"]
}