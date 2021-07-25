#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np


# In[2]:


######### 上 #########

head_str = "Camera Get Data Success\n"
end_str = "End of send first full image\n" 

whole_top = []
per_img = []
per_row = []

row_cnt = 0
img_cnt = 0
with open('teradata/top/top.txt', 'r') as f:
    for i, line in enumerate(f):
        if line != head_str:
            if line != end_str:
#                 per_row.append([int(n) for n in line.strip().split(',')])
#                 per_img.append(per_row)
#                 per_row = []
                per_img.append([int(n) for n in line.strip().split(',')])
                row_cnt = row_cnt + 1
                if(row_cnt == 60) : #放滿一張圖
                    row_cnt = 0
                    whole_top.append(per_img)
                    per_img = []
                    img_cnt = img_cnt + 1                
        if(img_cnt == 120) : #放滿所有圖
            print('finish')
            break
            
                    
print(len(whole_top))
print(len(per_img))
print(len(per_row))

arr_top = np.array(whole_top)
arr_top = np.int8(arr_top)
print(arr_top.shape)


# In[3]:


######### 下 #########
head_str = "Camera Get Data Success\n"
end_str = "End of send first full image\n" 

whole_top = []
per_img = []
per_row = []

row_cnt = 0
img_cnt = 0
with open('teradata/down/down.txt', 'r') as f:
    for i, line in enumerate(f):
        if line != head_str:
            if line != end_str:
#                 per_row.append([int(n) for n in line.strip().split(',')])
#                 per_img.append(per_row)
#                 per_row = []
                per_img.append([int(n) for n in line.strip().split(',')])
                row_cnt = row_cnt + 1
                if(row_cnt == 60) : #放滿一張圖
                    row_cnt = 0
                    whole_top.append(per_img)
                    per_img = []
                    img_cnt = img_cnt + 1                
        if(img_cnt == 120) : #放滿所有圖
            print('finish')
            break
            
                    
print(len(whole_top))
print(len(per_img))
print(len(per_row))

arr_down = np.array(whole_top)
arr_down = np.int8(arr_down)
print(arr_down.shape)


# In[4]:


######### 左 #########
head_str = "Camera Get Data Success\n"
end_str = "End of send first full image\n" 

whole_top = []
per_img = []
per_row = []

row_cnt = 0
img_cnt = 0
with open('teradata/left/left.txt', 'r') as f:
    for i, line in enumerate(f):
        if line != head_str:
            if line != end_str:
#                 per_row.append([int(n) for n in line.strip().split(',')])
#                 per_img.append(per_row)
#                 per_row = []
                per_img.append([int(n) for n in line.strip().split(',')])
                row_cnt = row_cnt + 1
                if(row_cnt == 60) : #放滿一張圖
                    row_cnt = 0
                    whole_top.append(per_img)
                    per_img = []
                    img_cnt = img_cnt + 1                
        if(img_cnt == 120) : #放滿所有圖
            print('finish')
            break
            
                    
print(len(whole_top))
print(len(per_img))
print(len(per_row))

arr_left = np.array(whole_top)
arr_left = np.int8(arr_left)
print(arr_left.shape)


# In[5]:


######### 右 #########
head_str = "Camera Get Data Success\n"
end_str = "End of send first full image\n" 

whole_top = []
per_img = []
per_row = []

row_cnt = 0
img_cnt = 0
with open('teradata/right/right.txt', 'r') as f:
    for i, line in enumerate(f):
        if line != head_str:
            if line != end_str:
#                 per_row.append([int(n) for n in line.strip().split(',')])
#                 per_img.append(per_row)
#                 per_row = []
                per_img.append([int(n) for n in line.strip().split(',')])
                row_cnt = row_cnt + 1
                if(row_cnt == 60) : #放滿一張圖
                    row_cnt = 0
                    whole_top.append(per_img)
                    per_img = []
                    img_cnt = img_cnt + 1                
        if(img_cnt == 120) : #放滿所有圖
            print('finish')
            break
            
                    
print(len(whole_top))
print(len(per_img))
print(len(per_row))

arr_right = np.array(whole_top)
arr_right = np.int8(arr_right)
print(arr_right.shape)


# In[6]:


######### 無 #########
head_str = "Camera Get Data Success\n"
end_str = "End of send first full image\n" 

whole_top = []
per_img = []
per_row = []

row_cnt = 0
img_cnt = 0
with open('teradata/no/no.txt', 'r') as f:
    for i, line in enumerate(f):
        if line != head_str:
            if line != end_str:
#                 per_row.append([int(n) for n in line.strip().split(',')])
#                 per_img.append(per_row)
#                 per_row = []
                per_img.append([int(n) for n in line.strip().split(',')])
                row_cnt = row_cnt + 1
                if(row_cnt == 60) : #放滿一張圖
                    row_cnt = 0
                    whole_top.append(per_img)
                    per_img = []
                    img_cnt = img_cnt + 1                
        if(img_cnt == 60) : #放滿所有圖
            print('finish')
            break
            
                    
print(len(whole_top))
print(len(per_img))
print(len(per_row))

arr_no = np.array(whole_top)
arr_no = np.int8(arr_no)
print(arr_no.shape)


# ## Dataset with Label

# In[7]:


X_data = np.concatenate((arr_top, arr_down, arr_left, arr_right, arr_no), axis=0)
print(X_data.shape)
print(X_data.dtype)


# In[8]:


totol_class = 5
y_data_list = []

for c in range(totol_class):
    if c < 4 :
        for i in range(120):
            y_data_list.append(c)
    else:
        for i in range(60):
            y_data_list.append(c)
            
y_data = np.array(y_data_list)
y_data = np.int8(y_data)
print(y_data.shape)
print(y_data.dtype)


# ## Train/Test split 

# In[9]:


# Train split  
from sklearn.model_selection import train_test_split
X_train, X_test, y_train, y_test = train_test_split(X_data, y_data, test_size=0.1, random_state=555, stratify=y_data)
print(X_train.shape)
print(X_test.shape)
print(y_train)
print(y_test)


# ## Image Preprocessing  and Model Building

# In[10]:


from tensorflow.keras.layers import Conv2D, MaxPooling2D, Dense
from tensorflow.keras.layers import Activation, BatchNormalization, Flatten
from tensorflow.keras.models import Sequential
from tensorflow.keras.utils import to_categorical

img_rows = 60
img_cols = 80
num_classes = 5
input_shape = (img_rows, img_cols, 1)
filter_x = 5
filter_y = 5

X_train = X_train.astype('float32')

train_images = X_train.reshape(X_train.shape[0], X_train.shape[1], X_train.shape[2], 1)
train_labels = to_categorical(y_train, num_classes, dtype='float32')
# 

print(train_images.shape)
print(train_labels)


# In[11]:


#model building
model = Sequential()

#Conv1
model.add(Conv2D(filters=16, 
                 kernel_size=(filter_x, filter_y), 
                 padding="same",  
                 input_shape=input_shape))
model.add(BatchNormalization())
model.add(Activation("relu"))
model.add(MaxPooling2D())

#Conv2
model.add(Conv2D(filters=32, 
                 kernel_size=(filter_x, filter_y), 
                 padding="same", 
                 input_shape=input_shape))
model.add(BatchNormalization())
model.add(Activation("relu"))
model.add(MaxPooling2D())

#Conv3
model.add(Conv2D(filters=32, 
                 kernel_size=(filter_x, filter_y), 
                 padding="same", 
                 input_shape=input_shape))
model.add(BatchNormalization())
model.add(Activation("relu"))
model.add(MaxPooling2D())

#FC1
model.add(Flatten())
model.add(Dense(64))
model.add(BatchNormalization())
model.add(Activation("relu"))

#FC2
model.add(Dense(num_classes))
model.add(Activation("softmax"))


model.compile(optimizer='adam',
              loss='categorical_crossentropy',
              metrics=['accuracy'])


print(model.summary())


# ## Training 

# In[12]:


# epochs = 20
# batch_size = 18

# model.fit(train_images, train_labels,
#           validation_split = 0.1,
#           batch_size = batch_size,
#           epochs = epochs,
#           verbose = 1,
#          )
# model.save_weights('himaxCam_model_0723.h5')


# In[13]:


model.load_weights('himaxCam_model_0723.h5')


# ## Testing

# In[14]:


# Test image = X_test (54, 60, 80)
# Test label = y_test (54, )
# shuffled in train_test_split
test_images = X_test.reshape(X_test.shape[0], X_test.shape[1], X_test.shape[2], 1)
test_labels = to_categorical(y_test, num_classes, dtype='float32')
print(test_images.shape)
print(test_labels.shape)


# In[15]:


score = model.evaluate(test_images, test_labels, verbose=1)
print('test Loss:     ', score[0])
print('test Accuracy: ', score[1])


# ## Convert model into TFLM format

# In[16]:


import tensorflow.compat.v2 as tf
converter = tf.lite.TFLiteConverter.from_keras_model(model) #a converted model with no quantization:
converter.optimizations = [tf.lite.Optimize.DEFAULT] #quantize weights into int8 other variables still in float format
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8  #quantize activations to int8
converter.inference_output_type = tf.int8 #


# In[17]:


preprocessed_train_images = train_images
preprocessed_train_images = tf.cast(preprocessed_train_images, tf.float32)
emnist_ds = tf.data.Dataset.from_tensor_slices((preprocessed_train_images)).batch(1) #construct a dataset 

def representative_data_gen():
    for input_value in emnist_ds.take(450): 
        yield [input_value]
    
converter.representative_dataset = representative_data_gen


# In[18]:


import pathlib

converted_model = converter.convert()

generated_dir = pathlib.Path("generated/")
generated_dir.mkdir(exist_ok=True, parents=True)
converted_model_file = generated_dir/"himaxCam_model_0723_int8.tflite"
converted_model_file.write_bytes(converted_model)


# ## Evaluate Model

# In[19]:


interpreter = tf.lite.Interpreter(model_path=str(converted_model_file))
interpreter.allocate_tensors()


# In[20]:


max_samples = 50
preprocessed_test_images = test_images
preprocessed_test_images = tf.cast(preprocessed_test_images, tf.float32)


# In[21]:


# A helper function to evaluate the TF Lite model using "test" dataset.
def evaluate_model(interpreter):
    input_index = interpreter.get_input_details()[0]["index"]
    output_index = interpreter.get_output_details()[0]["index"]
    scale, zero_point = interpreter.get_output_details()[0]['quantization']

    prediction_values = []
    
    for test_image in preprocessed_test_images[:max_samples]:
        # Pre-processing: add batch dimension, quantize and convert inputs to int8 to match with
        # the model's input data format.
        test_image = np.expand_dims(test_image, axis=0) #.astype(np.float32)
        test_image = np.int8(test_image)
#         test_image = np.int8(test_image / scale + zero_point)
        interpreter.set_tensor(input_index, test_image)
        interpreter.invoke()

        # Find the letter with highest probability
        output = interpreter.tensor(output_index)
        result = np.argmax(output()[0])
        prediction_values.append(result)

    accurate_count = 0
    for index in range(len(prediction_values)):
        for i in range(5):
            if test_labels[index][i]==1:
                y_class=i
        if (prediction_values[index] == y_class):
            accurate_count += 1
    accuracy = accurate_count * 1.0 / len(prediction_values)

    return accuracy * 100


# In[22]:


print(str(evaluate_model(interpreter)) + "%")


# ## Create a test set for target application

# In[23]:


import random
import matplotlib.pyplot as plt

num_of_samples = 25
random_test_images = random.sample(range(1, test_images.shape[0]), num_of_samples)
print(test_images[0].shape)
fig=plt.figure(figsize=(5, 5))
cols = 5
rows = 5

for plt_idx, img_idx in enumerate(random_test_images, 1):
    img = test_images[img_idx]
    fig.add_subplot(rows, cols, plt_idx)
    plt.imshow(img)
plt.show()


# In[24]:


samples_file = open("generated/test_samples.cc", "w")

samples_file.write("#include \"test_samples.h\"\n\n")
samples_file.write("const int kNumSamples = " + str(num_of_samples) + ";\n\n")

samples = "" 
samples_array = "const TestSample test_samples[kNumSamples] = {"

for sample_idx, img_idx in enumerate(random_test_images, 1):
    img_arr = list(np.ndarray.flatten(test_images[img_idx]))
    var_name = "sample" + str(sample_idx)
    samples += "TestSample " + var_name + " = {\n" #+ "[IMAGE_SIZE] = { "
    samples += "\t.label = " + str(test_labels[img_idx]) + ",\n" 
    samples += "\t.image = {\n"
    wrapped_arr = [img_arr[i:i + 20] for i in range(0, len(img_arr), 20)]
    for sub_arr in wrapped_arr:
        samples += "\t\t" + str(sub_arr)
    samples += "\t}\n};\n\n"    
    samples_array += var_name + ", "
    
samples = samples.replace("[", "")
samples = samples.replace("]", ",\n")
samples_array += "};\n"

samples_file.write(samples);
samples_file.write(samples_array);
samples_file.close()

