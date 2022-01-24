import pandas as pd
import numpy as np

def extract_training_labels_csv(csv_object):
    """
    We suppose that in the test and train image sets there are all the labels
    """
    unique_label_set = set([])
    d = csv_object.loc[: , "class"]
    for key,value in d.iteritems():
        unique_label_set.add(value)
    
    # We convert to have functions like sort
    unique_label_list = list(unique_label_set)
    unique_label_list.sort()
    return unique_label_list

def class_text_to_int(row_label, unique_label_array):
    """
    The labels will be assigned by alpabetical order
    This is very importatnt for the config file for training
    """
    class_integer = int(unique_label_array.index(row_label) + 1)
    #print ("Label="+str(row_label)+", Index="+str(class_integer))
    return class_integer
    

if __name__ == "__main__":
    print("Opening CSV...")
    csv_input_for_labels = "scripts/dummy1.csv"
    examples = pd.read_csv(csv_input_for_labels)
    print("Opened CSV...")
    unique_label_array = extract_training_labels_csv(examples)
    label_contents = ""
    for lable in unique_label_array:
        print("Generating Index for lable=="+str(lable))
        index = class_text_to_int(lable, unique_label_array)
        print("Label=="+str(lable)+", index ="+str(index))
        label_contents += "item {\n    id : "+str(index)+"\n    name : "+str(lable)+"\n}\n"