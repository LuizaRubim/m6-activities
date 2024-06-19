from keras.datasets import mnist
from keras.optimizers import Adam

# Carregando o dataset separando os dados de treino e de teste 
(x_treino, y_treino), (x_teste, y_teste) = mnist.load_data()

# Permite imprimir dentro do notebook Python
# import matplotlib.pyplot as plt
# %matplotlib inline

# Exemplo de um dado do dataset
# plt.imshow(x_treino[4], cmap='gray_r')

# Trazendo a função `to_categorical` para transformar os labels em one-hot encoding
from keras.utils import to_categorical
y_treino_cat = to_categorical(y_treino)
y_teste_cat = to_categorical(y_teste)

# Verificação da saída one-hot encoding
print(y_treino[0]) #Valor da classe
print(y_treino_cat[0]) #Representação onehot

# Normalização dos dados de entrada
x_treino_norm = x_treino/x_treino.max()
x_teste_norm = x_teste/x_teste.max()

# Reshape dos dados de entrada para adicionar o canal de cor
x_treino = x_treino.reshape(len(x_treino), 28, 28, 1)
x_treino_norm = x_treino_norm.reshape(len(x_treino_norm), 28, 28, 1)
x_teste = x_teste.reshape(len(x_teste), 28, 28, 1)
x_teste_norm = x_teste_norm.reshape(len(x_teste_norm), 28, 28, 1)

# Importação das bibliotecas necessárias para treinar a rede
#Modelo da rede
from keras.models import Sequential
#Camadas que serão utilizadas
from keras.layers import Dense, Conv2D, MaxPool2D, Flatten, Dropout

# Criação do modelo LeNet5
model = Sequential()
model.add(Conv2D(filters=32, kernel_size=(5,5), padding='same', activation='relu', input_shape=(28, 28, 1)))
model.add(MaxPool2D(strides=2))
model.add(Conv2D(filters=48, kernel_size=(5,5), padding='valid', activation='relu'))
model.add(MaxPool2D(strides=2))
model.add(Flatten())
model.add(Dense(256, activation='relu'))
model.add(Dense(84, activation='relu'))
model.add(Dense(10, activation='softmax'))

# Constroi o modelo
model.build()
# Exibe um resumo do modelo
model.summary()

#Compila o modelo
adam = Adam()
model.compile(loss='categorical_crossentropy', metrics=['accuracy'], optimizer=adam)

# Realiza o treinamento do modelo
historico = model.fit(x_treino_norm, y_treino_cat, epochs=5, validation_split=0.2)

# Exibe o histórico do treinamento
# Graficos de perda e acurácia
# Acurácia
# plt.plot(historico.history['accuracy'])
# plt.plot(historico.history['val_accuracy'])
# plt.legend(['treino', 'validacao'])
# plt.xlabel('epocas')
# plt.ylabel('acuracia')

# # Perda
# plt.plot(historico.history['loss'])
# plt.plot(historico.history['val_loss'])
# plt.legend(['treino', 'validacao'])
# plt.xlabel('epocas')
# plt.ylabel('perda')

# Salva o modelo
model.save('modelo_mnist.h5')

# Carrega o modelo
from tensorflow.keras.models import load_model
modelo_2 = load_model('modelo_mnist.h5')

# Realiza uma predição com o modelo
predicao = model.predict(x_teste_norm[0].reshape(1, 28, 28, 1))
print(predicao)
# Exibe a classe com a maior probabilidade de ser a correta da predição
import numpy as np
np.argmax(predicao)
