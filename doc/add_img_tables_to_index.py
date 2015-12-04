#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Script to add image table with link to document
to index file of sphinx documentation.

Usage::

    # in conf.py for sphinx doc
    exclude_patterns = ['_build', 'venv']
    import add_img_tables_to_index
    add_img_tables_to_index.main(exclude_patterns)
"""

import hashlib
import os
import sys


__copyright__ = '2015, JSK Lab'
__author__ = 'Kentaro Wada <www.kentaro.wada@gmail.com>'
__version__ = '0.1'


def rst_image_table_data(url_prefix, img_files):
    N_COLUMN = 3
    # collect label and blocks
    labels = []
    blocks = []
    for i, img_file in enumerate(img_files):
        title, _ = os.path.splitext(img_file)
        doc_candidates = [os.path.join(url_prefix, title + ext)
                          for ext in ['.rst', '.md']]
        if not any(map(os.path.exists, doc_candidates)):
            continue
        img_file = os.path.join(url_prefix, 'images', img_file)
        label_name = hashlib.sha1(title).hexdigest()[:8]
        labels.append('''\
.. |{label}| image:: {img}
   :scale: 100%
   :align: middle
   :target: {url}'''.format(label=label_name, img=img_file,
                            url=os.path.join(url_prefix, title + '.html')))
        blocks.append('|{label}|'.format(label=label_name))
    # pack blocks with blank text
    while len(blocks) % N_COLUMN != 0:
        blocks.append(' ' * 10)
    # generate table
    table = []
    table.append('+------------+------------+------------+')
    for i in xrange(0, len(blocks), N_COLUMN):
        table.append('| ' + ' | '.join(blocks[i:i+N_COLUMN]) + ' |')
        table.append('+------------+------------+------------+')
    return '\n\n'.join(labels), '\n'.join(table)


def add_img_table_to_index(pkg):
    cwd = os.getcwd()
    os.chdir(pkg)

    sub_dirs = [d for d in os.listdir('.') if os.path.isdir(d)]
    for sub in sub_dirs:
        if not os.path.exists(os.path.join(sub, 'images')):
            continue
        img_files = os.listdir(os.path.join(sub, 'images'))
        img_files = [f for f in img_files
                     if os.path.isfile(os.path.join(sub, 'images', f))]
        labels, table = rst_image_table_data(sub, img_files)
        with open('index.rst', 'a') as f:
            f.write('\n\n')
            f.write(labels)
            f.write('\n\n\n')
            f.write(table)

    os.chdir(cwd)


def main(exclude_patterns):
    pkgs = [d for d in os.listdir('.') if os.path.isdir(d)]
    pkgs = [d for d in pkgs if d not in exclude_patterns]
    for pkg in pkgs:
        add_img_table_to_index(pkg)


if __name__ == '__main__':
    main()
