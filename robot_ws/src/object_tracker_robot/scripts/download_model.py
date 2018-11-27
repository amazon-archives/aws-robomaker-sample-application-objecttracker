#!/usr/bin/env python

# This is a build-time script used to download a file
# from S3 and place it in a desired location. It is invoked
# by CMake during build time.

import boto3
import sys
import yaml

def load_config(config_path):
    stream = file(config_path, 'r')
    context = yaml.safe_load(stream)

    required_items = (
        'bucket_name',
        'bucket_prefix',
        'model_file',
        'aws_access_key_id',
        'aws_secret_access_key',
        'aws_session_token',
        'region_name'
    )

    if not all(item in context for item in required_items):
        raise ValueError('Config file is missing required items')

    return context

def download_model_from_s3(context, dest_path):
    s3_bucket = context['bucket_name']
    s3_key = context['bucket_prefix'] + '/' + context['model_file']

    s3 = boto3.resource('s3',
        region_name=context['region_name'],
        aws_access_key_id=context['aws_access_key_id'],
        aws_secret_access_key=context['aws_secret_access_key'],
        aws_session_token=context['aws_session_token'])
    s3.meta.client.download_file(
        Bucket=context['bucket_name'],
        Key=context['bucket_prefix'] + '/' + context['model_file'],
        Filename=dest_path)

def main(argv):
    if len(argv) != 3:
        raise ValueError('Model downloader usage: python model_downloader.py config_file_path dest_path')

    source_config = argv[1]
    dest_path = argv[2]

    context = load_config(source_config)

    download_model_from_s3(context, dest_path)

    print 'Successfully downloaded model'

if __name__ == '__main__':
    main(sys.argv)
