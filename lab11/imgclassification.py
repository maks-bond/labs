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
        i = 0
        for im in data:
            gray_im = color.rgb2gray(im)
            gray_blurred = filters.gaussian(gray_im, sigma=1)
            # thresh = filters.threshold_otsu(gray_blurred)
            # binary_im = gray_blurred > thresh
            # binary_blurred = filters.gaussian(binary_im, sigma=1)

            # thresh = filters.threshold_li(gray_blurred)
            # binary_im = gray_blurred > thresh

            #fig, ax = filters.try_all_threshold(gray_blurred, figsize=(10, 8), verbose=False)
            #plt.show()

            # io.imshow(binary_im)
            # io.show()
            # break
            #gray_adjusted_im = exposure.adjust_gamma(gray_blurred, gamma=0.5, gain=1)
            #local_otsu = filters.rank.otsu(gray_adjusted_im, disk(5))
            thresh = filters.threshold_otsu(gray_blurred)
            binary_im = gray_blurred >= thresh *0.75
            # if i == 34:
            #     binary_im = gray_blurred >= thresh * 0.6
            # fig, ax = filters.try_all_threshold(gray_adjusted_im, figsize=(10, 8), verbose=False)
            # plt.show()
            # if i == 34:
            #     io.imshow(binary_im)
            #     io.show()
            i+=1
            (hog_features, hog_im) = feature.hog(binary_im, orientations=orientations, pixels_per_cell = pixels_per_cell, cells_per_block = cells_per_block,
                                                 block_norm=norm, visualise=True, feature_vector=True,
                                                 transform_sqrt=True)
            feature_data.append(hog_features)
        return(feature_data)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above
        
        # train model and save the trained model to self.classifier
        
        ########################
        ######## YOUR CODE HERE
        ########################
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
        predicted_labels = self.classifier.predict(data)
        return predicted_labels

# TODO: Do nearest neighbors
# TODO: threshold_otsu
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
    print("Not matched labels")
    print("Indexes:")
    print(np.where(test_labels != predicted_labels))
    print("Test Labels:")
    print(test_labels[np.where(test_labels != predicted_labels)])
    print("Predicted Labels:")
    print(predicted_labels[np.where(test_labels != predicted_labels)])


if __name__ == "__main__":
    main()
