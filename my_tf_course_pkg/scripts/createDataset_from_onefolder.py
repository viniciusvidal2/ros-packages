import os
import random
import copy
from shutil import copyfile
from shutil import rmtree


def generate_list_files_dataset_clean(dataset_dir_path = "./new_dataset"):

	images_list, annotation_list = check_annotations_vs_images(dataset_dir_path, assert_active=False)

	for i in images_list[:]:
	    image_name_xml = os.path.splitext(i)[0]+".xml"
	    if image_name_xml not in annotation_list:
		    images_list.remove(i)

	for j in annotation_list[:]:
	    annotation_name_jpg = os.path.splitext(j)[0]+".jpg"
	    annotation_name_png = os.path.splitext(j)[0]+".png"
	    if annotation_name_jpg not in images_list and annotation_name_png not in images_list:
		    annotation_list.remove(j)

	annotation_length = len(annotation_list)
	images_length = len(images_list)
	print("Annotations="+str(annotation_length))
	print("Images="+str(images_length))

	if annotation_length != images_length:
	    assert False,"Lengths are note the same...Something is wrong"
	else:
	    print("Lengths ok and CLEAN")

	return annotation_list, images_list


def divide_dataset(image_list, ratio=0.8):

	train_image_list = []
	test_image_list = []
	len_image_list = len(image_list)
	len_train_image_list = int(len_image_list * ratio)
	len_test_image_list = len_image_list - len_train_image_list
	assert (len_train_image_list + len_test_image_list) == len_image_list, "Total Length wrong"
	# We do it of the test images because its always smaller thn the train images list
	for i in range(len_test_image_list):
		test_image = random.choice(image_list)
		image_list.remove(test_image)
		test_image_list.append(test_image)

	train_image_list = copy.deepcopy(image_list)

	len_train_image_list = int(len(train_image_list))
	len_test_image_list = int(len(test_image_list))

	print("len_train_image_list="+str(len_train_image_list))
	print("len_test_image_list="+str(len_test_image_list))

	assert (len_train_image_list + len_test_image_list) == len_image_list, "Total Length wrong AFTER Generation"

	return train_image_list, test_image_list

def dataset_copy_files_to_folders(train_image_list, test_image_list, dataset_dir_path = "./new_dataset", train_test_dataset_dir_path = "./new_dataset"):

	abs_dataset_dir_path = os.path.abspath(dataset_dir_path)
	train_images_folder = os.path.abspath(os.path.join(train_test_dataset_dir_path, "train"))
	validation_images_folder = os.path.abspath(os.path.join(train_test_dataset_dir_path, "test"))

	copy_files_to_folder(abs_dataset_dir_path, train_images_folder, train_image_list)
	copy_files_to_folder(abs_dataset_dir_path, validation_images_folder, test_image_list)

	print("Checking the train folder files")
	check_list_files(train_image_list, train_images_folder)

	print("Checking the test folder files")
	check_list_files(test_image_list, validation_images_folder)


def copy_files_to_folder(origin_folder, destination_folder, image_list):

	# We clean up the training folders
	if os.path.exists(destination_folder):
		rmtree(destination_folder)
	
	os.makedirs(destination_folder)
	print("Created folder=" + str(destination_folder))

	for image_file_name in image_list:
		image_file_name_xml = os.path.splitext(image_file_name)[0]+".xml"
		abs_path_file = os.path.abspath(os.path.join(origin_folder,image_file_name))
		abs_path_file_xml = os.path.abspath(os.path.join(origin_folder,image_file_name_xml))

		abs_path_file_destination = os.path.abspath(os.path.join(destination_folder,image_file_name))
		abs_path_file_destination_xml = os.path.abspath(os.path.join(destination_folder,image_file_name_xml))
		
		copyfile(abs_path_file, abs_path_file_destination)
		copyfile(abs_path_file_xml, abs_path_file_destination_xml)


def check_annotations_vs_images(folder_path, assert_active=False):	
	annotation_list= []
	images_list = []
	for file_name in os.listdir(folder_path):
	    if file_name.endswith(".xml"):
		    annotation_list.append(file_name)
	    if file_name.endswith(".jpg") or file_name.endswith(".png"):
		    images_list.append(file_name)

	annotation_length = len(annotation_list)
	images_length = len(images_list)
	print("Annotations="+str(annotation_length))
	print("Images="+str(images_length))

	if annotation_length != images_length:
		msg = "Lengths are note the same...Something is wrong"
		if assert_active:
			assert False,msg
		else:
			print(msg)
	else:
		print("Lengths ok and CLEAN")

	return images_list, annotation_list


def check_list_files(list_files, folder_path):

	images_list, annotation_list = check_annotations_vs_images(folder_path)

	if len(list_files) != len(images_list):
		assert False, "The images in the folder dont coincide"
	else:
		print("Number of files n folder and in list are the same, so everything OK")

		


annotation_list, images_list = generate_list_files_dataset_clean()
train_image_list, test_image_list = divide_dataset(images_list, ratio=0.8)
dataset_copy_files_to_folders(train_image_list, test_image_list, dataset_dir_path = "./new_dataset", train_test_dataset_dir_path = "./new_dataset")