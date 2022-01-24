from __future__ import division
from __future__ import print_function
from __future__ import absolute_import

"""
Usage:
  # From tensorflow/models/
  # Create train data:
  python generate_tfrecord.py --csv_input=data/train_labels.csv  --output_path=data/train.record

  # Create test data:
  python generate_tfrecord.py --csv_input=data/test_labels.csv  --output_path=data/test.record
"""

import os
import io
import pandas as pd
import numpy as np
import tqdm
import tensorflow as tf

from PIL import Image
from collections import namedtuple, OrderedDict
from extract_training_lables_csv import extract_training_labels_csv, class_text_to_int

from absl import app, flags, logging
from absl.flags import FLAGS
import hashlib

"""
flags = tf.app.flags
flags.DEFINE_string('image_path_input', '', 'Path to the Images refered to in the CSV')
flags.DEFINE_string('csv_input', '', 'Path to the CSV input')
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
FLAGS = flags.FLAGS
"""
FLAGS = flags.FLAGS
flags.DEFINE_string('image_path_input', '', 'Path to the Images refered to in the CSV')
flags.DEFINE_string('csv_input', '', 'Path to the CSV input')
flags.DEFINE_string('output_path', '', 'Path to output TFRecord')
flags.DEFINE_string('output_path_names', '', 'Path to output New Class Names file')


def split(df, group):
    data = namedtuple('data', ['filename', 'object'])
    gb = df.groupby(group)
    return [data(filename, gb.get_group(x)) for filename, x in zip(gb.groups.keys(), gb.groups)]


def create_tf_example(group, path, unique_label_array):
    """
    with tf.gfile.GFile(os.path.join(path, '{}'.format(group.filename)), 'rb') as fid:
        encoded_jpg = fid.read()
    encoded_jpg_io = io.BytesIO(encoded_jpg)
    image = Image.open(encoded_jpg_io)
    """

    img_path = os.path.join(path, '{}'.format(group.filename))
    image = open(img_path, 'rb').read()
    key = hashlib.sha256(image).hexdigest()

    filename = group.filename.encode('utf8')
    image_format = b'jpg'
    xmins = []
    xmaxs = []
    ymins = []
    ymaxs = []
    classes_text = []
    classes = []
    truncated = []
    views = []
    difficult_obj = []

    

    # The original labeling didnt have this
    truncated_hardcoded = 0
    difficult_hardcoded = 0
    view_hardcoded = "Unspecified"

    for index, row in group.object.iterrows():
        width = int(row['width'])
        height = int(row['height'])
        xmins.append(float(row['xmin'] / width))
        xmaxs.append(float(row['xmax'] / width))
        ymins.append(float(row['ymin'] / height))
        ymaxs.append(float(row['ymax'] / height))
        classes_text.append(row['class'].encode('utf8'))
        classes.append(class_text_to_int(row['class'], unique_label_array))

        difficult = bool(int(difficult_hardcoded))
        difficult_obj.append(int(difficult))
        
        truncated.append(int(truncated_hardcoded))
        views.append(view_hardcoded.encode('utf8'))


    tf_example = tf.train.Example(features=tf.train.Features(feature={
        'image/height': tf.train.Feature(int64_list=tf.train.Int64List(value=[height])),
        'image/width': tf.train.Feature(int64_list=tf.train.Int64List(value=[width])),
        'image/filename': tf.train.Feature(bytes_list=tf.train.BytesList(value=[filename])),
        'image/source_id': tf.train.Feature(bytes_list=tf.train.BytesList(value=[filename])),
        'image/key/sha256': tf.train.Feature(bytes_list=tf.train.BytesList(value=[key.encode('utf8')])),
        'image/encoded': tf.train.Feature(bytes_list=tf.train.BytesList(value=[image])),
        'image/format': tf.train.Feature(bytes_list=tf.train.BytesList(value=['jpeg'.encode('utf8')])),
        'image/object/bbox/xmin': tf.train.Feature(float_list=tf.train.FloatList(value=xmins)),
        'image/object/bbox/xmax': tf.train.Feature(float_list=tf.train.FloatList(value=xmaxs)),
        'image/object/bbox/ymin': tf.train.Feature(float_list=tf.train.FloatList(value=ymins)),
        'image/object/bbox/ymax': tf.train.Feature(float_list=tf.train.FloatList(value=ymaxs)),
        'image/object/class/text': tf.train.Feature(bytes_list=tf.train.BytesList(value=classes_text)),
        'image/object/class/label': tf.train.Feature(int64_list=tf.train.Int64List(value=classes)),        
        'image/object/difficult': tf.train.Feature(int64_list=tf.train.Int64List(value=difficult_obj)),
        'image/object/truncated': tf.train.Feature(int64_list=tf.train.Int64List(value=truncated)),
        'image/object/view': tf.train.Feature(bytes_list=tf.train.BytesList(value=views)),
        
    }))


    return tf_example

def create_class_names_file(unique_class_list, names_file_path="./new_class_names.names"):
    print("Generating New Names Class list file==>"+str(names_file_path))
    with open(names_file_path, 'w') as f:
        for item in unique_class_list:
            f.write("%s\n" % item)
            print("New Class to file==>"+str(item))
    print("Generating New Names Class list file...DONE")


def main(_argv):
    #writer = tf.python_io.TFRecordWriter(FLAGS.output_path)
    writer = tf.io.TFRecordWriter(FLAGS.output_path)

    path = os.path.join(os.getcwd(), FLAGS.image_path_input)
    examples = pd.read_csv(FLAGS.csv_input)
    unique_label_array = extract_training_labels_csv(examples)
    
    create_class_names_file(unique_class_list=unique_label_array,
                            names_file_path=FLAGS.output_path_names)
    grouped = split(examples, 'filename')
    for group in tqdm.tqdm(grouped):
    #for group in grouped:
        tf_example = create_tf_example(group, path, unique_label_array)
        writer.write(tf_example.SerializeToString())

    writer.close()
    output_path = os.path.join(os.getcwd(), FLAGS.output_path)
    print('Successfully created the TFRecords: {}'.format(output_path))


if __name__ == '__main__':
    #tf.app.run()
    app.run(main)