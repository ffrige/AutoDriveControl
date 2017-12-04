import cv2
import os
import csv
import numpy as np
import sklearn

from keras.models import Sequential, Model
from keras.layers.core import Dense, Activation, Flatten, Dropout
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D
from keras.layers import Cropping2D, Lambda
from keras.utils import to_categorical

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle


#3 possible output classes: -1,0,+1
# left, straight, right
NUM_CLASSES = 3


# pre-process input image
# turn gray
# thresholds for vertical gradient + gradient magnitude + gradient direction
# output is 1 layer binary image
def pre_process(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1)

    abs_sobel =  np.absolute(sobel_y)
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))

    gradmag = np.sqrt(sobel_x**2 + sobel_y**2)
    scale_factor = np.max(gradmag)/255
    gradmag = (gradmag/scale_factor).astype(np.uint8) 

    absgraddir = np.arctan2(np.absolute(sobel_y), np.absolute(sobel_x))

    binary_output = np.zeros_like(gradmag)

    binary_output[(gradmag >= 50) & (gradmag <= 120)
                  & (absgraddir >= 0.7) & (absgraddir <= 2)
                  & (scaled_sobel >= 50) & (scaled_sobel <= 120)] = 255

    binary_output[:70,:] = 0
    binary_output[-50:,:] = 0
    binary_output = np.uint8(binary_output)

    return binary_output.reshape(160,320,1)


def generator(samples, batch_size=32):
    num_samples = len(samples)
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]

            images = []
            angles = []
            for batch_sample in batch_samples:
                name = '/data/IMG/'+batch_sample[0].split('\\')[-1]
                center_image = cv2.imread(name)
                center_angle = float(batch_sample[3])

                #randomly flip images to augment data
                if (np.random.random()>0.5):
                    center_image = np.fliplr(center_image)
                    center_angle = -center_angle
                    
                images.append(pre_process(center_image))

                #3 possible classes: -1,0,+1
                center_angle = np.clip(np.trunc(center_angle*2.),-1,1)+1
                angles.append(center_angle)

            X_train = np.array(images)
            y_train = np.array(angles)
            y_train = to_categorical(y_train, num_classes=NUM_CLASSES)

            yield sklearn.utils.shuffle(X_train, y_train)



if __name__ == '__main__':

    samples = []
    with open('/data/driving_log.csv') as csvfile:
        reader = csv.reader(csvfile)
        for line in reader:
            samples.append(line)

    train_samples, validation_samples = train_test_split(samples, test_size=0.2)

    
    ### DID NOT USE GENERATOR on FloydHub ###
    # compile and train the model using the generator function
    #train_generator = generator(train_samples, batch_size=32)
    #validation_generator = generator(validation_samples, batch_size=32)

    
    ### DID NOT USE GENERATOR on FloydHub ###
    # load all images from memory 
    images = []
    angles = []
    shuffle(samples)
    for sample in samples:
        name = '/data/IMG/'+sample[0].split('\\')[-1]
        center_image = cv2.imread(name)
        center_angle = float(sample[3])

        #randomly flip images to augment data
        if (np.random.random()>0.5):
            center_image = np.fliplr(center_image)
            center_angle = -center_angle
                    
        images.append(pre_process(center_image))

        #3 possible classes: -1,0,+1
        center_angle = np.clip(np.trunc(center_angle*2.),-1,1)+1
        angles.append(center_angle)

    X_train = np.array(images)
    y_train = np.array(angles)
    y_train = to_categorical(y_train, num_classes=NUM_CLASSES)


    # define model here
    # input -> crop image -> normalize -> 2 Conv layers ->
    # -> 2 fully connected layers -> output softmax to 3 classes
    model = Sequential()
    model.add(Cropping2D(cropping=((70,50), (0,0)), input_shape=(160,320,1)))
    model.add(Lambda(lambda x: x/255. - 0.5)) #normalize

    model.add(Conv2D(32,(3,3), activation='relu')) #input shape here is (40,320,1) 
    model.add(MaxPooling2D((2, 2)))
    model.add(Dropout(0.5))

    model.add(Conv2D(64,(3,3), activation='relu'))
    model.add(MaxPooling2D((2, 2)))
    model.add(Dropout(0.5))

    model.add(Flatten())
    model.add(Dense(128, activation='relu'))
    model.add(Dense(64, activation='relu'))

    model.add(Dense(NUM_CLASSES, activation='softmax'))

    model.compile(loss='categorical_crossentropy', optimizer='adam', metrics=['accuracy'])
    model.fit(X_train,y_train,verbose=2,epochs=5,validation_split=0.2)

    ### DID NOT USE GENERATOR on FloydHub ###
    #model.fit_generator(train_generator, validation_data=validation_generator,
    #            validation_steps=len(validation_samples), epochs=5, steps_per_epoch= len(train_samples), verbose=2)

    model.save('/output/model_car.h5')
