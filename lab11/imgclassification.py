#!/usr/bin/env python

##############
#### Your name:
##############

import numpy as np
import re
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color
from skimage.morphology import disk
import matplotlib.pyplot as plt
import time

class ImageClassifier:
    
    def __init__(self):
        self.classifier = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)
        
        #create one large array of image data
        data = io.concatenate_images(ic)
        
        #extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]
        
        return(data,labels)

    def extract_image_features(self, data):
        # Please do not modify the header above

        # extract feature vector from image data

        ########################
        ######## YOUR CODE HERE
        ########################
        
        # Please do not modify the return type below
        feature_data = []
        pixels_per_cell = (20, 20)
        cells_per_block = (2, 2)
        # TODO: Downsize images?
        norm = "L1-sqrt"
        orientations = 4
        for im in data:
            # Convert to gray
            gray_im = color.rgb2gray(im)
            # Blur a bit to reduce noise
            gray_blurred = filters.gaussian(gray_im, sigma=1)
            # Apply otsu threshold
            thresh = filters.threshold_otsu(gray_blurred)
            # Filter using fraction of a threshold to get binary image
            binary_im = gray_blurred >= thresh *0.75
            # Get hog features
            hog_features = feature.hog(binary_im, orientations=orientations, pixels_per_cell = pixels_per_cell, cells_per_block = cells_per_block,
                                                 block_norm=norm, visualise=False, feature_vector=True,
                                                 transform_sqrt=True)
            feature_data.append(hog_features)
        return(feature_data)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above
        
        # train model and save the trained model to self.classifier
        
        ########################
        ######## YOUR CODE HERE
        ########################

        # Fit linear SVM classifier
        self.classifier = svm.LinearSVC()
        self.classifier.fit(train_data, train_labels)

    def predict_labels(self, data):
        # Please do not modify the header

        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels
        
        ########################
        ######## YOUR CODE HERE
        ########################
        
        # Please do not modify the return type below

        # Predict using trained linear svm
        predicted_labels = self.classifier.predict(data)
        return predicted_labels

def main():

    img_clf = ImageClassifier()

    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')

    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    test_data = img_clf.extract_image_features(test_raw)
    
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)
    predicted_labels = img_clf.predict_labels(train_data)
    print("\nTraining results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(train_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(train_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(train_labels, predicted_labels, average='micro'))
    
    # test model
    predicted_labels = img_clf.predict_labels(test_data)
    print("\nTest results")
    print("=============================")
    print("Confusion Matrix:\n",metrics.confusion_matrix(test_labels, predicted_labels))
    print("Accuracy: ", metrics.accuracy_score(test_labels, predicted_labels))
    print("F1 score: ", metrics.f1_score(test_labels, predicted_labels, average='micro'))

    # Get those images which didn't match
    # print("Not matched labels:")
    # print("Indexes:")
    # print(np.where(test_labels != predicted_labels))
    # print("Test Labels:")
    # print(test_labels[np.where(test_labels != predicted_labels)])
    # print("Predicted Labels:")
    # print(predicted_labels[np.where(test_labels != predicted_labels)])


if __name__ == "__main__":
    main()
