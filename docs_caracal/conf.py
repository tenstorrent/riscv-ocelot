# -*- coding: utf-8 -*-
#
# Configuration file for the Sphinx documentation builder (Caracal architecture).
# https://www.sphinx-doc.org/en/master/config

# -- Project information -----------------------------------------------------

project = 'Caracal'
copyright = 'Tenstorrent'
author = 'Tenstorrent'
version = ''
release = ''

# -- General configuration ---------------------------------------------------

extensions = [
    'sphinx.ext.autosectionlabel',
]

# Make autosectionlabel keys unique by prefixing with the document name.
autosectionlabel_prefix_document = True

templates_path = ['_templates']
source_suffix = '.rst'

# The docs are split across src/*.rst; index.rst is the landing page whose
# toctree pulls them together.
master_doc = 'index'

language = 'en'
# Exclude build/venv dirs and the legacy single-file source (its content now
# lives in index.rst + src/*.rst) so they don't trigger stray-source or
# "not included in any toctree" warnings.
exclude_patterns = [
    '_build',
    '.venv',
    'env',
    'caracal-architecture.rst',
    'README',
    'Thumbs.db',
    '.DS_Store',
]
pygments_style = 'sphinx'

# Number figures, tables and code-blocks.
numfig = True

# -- Options for HTML output -------------------------------------------------

html_theme = 'sphinx_rtd_theme'
# No custom static assets yet; figures live under figures/ and are referenced
# directly. Add '_static' back here (and create the dir) if you add CSS/JS.
html_static_path = []
htmlhelp_basename = 'Caracaldoc'

# -- Options for LaTeX output ------------------------------------------------

latex_elements = {}
latex_documents = [
    (master_doc, 'caracal-architecture.tex', 'The Caracal Microarchitecture',
     'Tenstorrent', 'manual'),
]

# -- Options for rinohtype PDF output (pure-Python, no LaTeX) -----------------
# Enables `make pdf` / `sphinx-build -b rinoh`.
rinoh_documents = [dict(
    doc=master_doc,
    target='caracal-architecture',
    title='The Caracal Microarchitecture',
    author='Tenstorrent',
)]
