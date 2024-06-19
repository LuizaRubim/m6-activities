import numpy as np
from keras.datasets import mnist
from keras.optimizers import Adam
from keras.utils import to_categorical
from keras.models import Sequential
from keras.layers import Dense, Conv2D, MaxPool2D, Flatten, Dropout
from tensorflow.keras.models import load_model

(x_treino, y_treino), (x_teste, y_teste) = mnist.load_data()

y_treino_cat = to_categorical(y_treino)
y_teste_cat = to_categorical(y_teste)

x_treino_norm = x_treino/x_treino.max()
x_teste_norm = x_teste/x_teste.max()

x_treino = x_treino.reshape(len(x_treino), 28, 28, 1)
x_treino_norm = x_treino_norm.reshape(len(x_treino_norm), 28, 28, 1)
x_teste = x_teste.reshape(len(x_teste), 28, 28, 1)
x_teste_norm = x_teste_norm.reshape(len(x_teste_norm), 28, 28, 1)

model = Sequential()
model.add(Conv2D(filters=32, kernel_size=(5,5), padding='same', activation='relu', input_shape=(28, 28, 1)))
model.add(MaxPool2D(strides=2))
model.add(Conv2D(filters=48, kernel_size=(5,5), padding='valid', activation='relu'))
model.add(MaxPool2D(strides=2))
model.add(Flatten())
model.add(Dense(256, activation='relu'))
model.add(Dense(84, activation='relu'))
model.add(Dense(10, activation='softmax'))

model.build()
model.summary()

adam = Adam()
model.compile(loss='categorical_crossentropy', metrics=['accuracy'], optimizer=adam)

historico = model.fit(x_treino_norm, y_treino_cat, epochs=50, validation_split=0.2)

model.save('modelo_mnist.h5')

modelo_2 = load_model('modelo_mnist.h5')

predicao = model.predict(x_teste_norm[0].reshape(1, 28, 28, 1))
print(predicao)

np.argmax(predicao)
