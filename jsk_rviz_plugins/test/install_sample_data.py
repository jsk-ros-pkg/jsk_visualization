#!/usr/bin/env python

import argparse
import multiprocessing

import jsk_data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('-v', '--verbose', dest='quiet', action='store_false')
    args = parser.parse_args()
    quiet = args.quiet

    def download_data(**kwargs):
        kwargs['pkg_name'] = 'jsk_rviz_plugins'
        kwargs['quiet'] = quiet
        p = multiprocessing.Process(
            target=jsk_data.download_data,
            kwargs=kwargs)
        p.start()

    download_data(
        path='test/data/2017-06-20-12-00-00_people_images_in_lab.bag.tgz',
        url='https://drive.google.com/uc?id=1VYLgjccB9sCa5ht32r3FLjUC2fhFkyZI',
        md5='0a397a2ff14c07954cc0b9178e33600d',
        extract=True,
    )


if __name__ == '__main__':
    main()
