from flask import Flask, request, render_template, render_template_string
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import os

modelo = load_model('modelo_mnist.h5')

app = Flask(__name__)
app.config["TEMPLATES_AUTO_RELOAD"] = True

@app.route('/', methods=['GET'])
def home():
    return render_template('index.html')

@app.route('/modelo', methods=['POST'])
def model():
    file = request.files['teste_modelo']
    
    filepath = os.path.join('temp', "imagem.png")
    file.save(filepath)
    
    img = cv2.imread(filepath, cv2.IMREAD_GRAYSCALE)
    img = cv2.resize(img, (28, 28))
    img = img / img.max()
    _, img = cv2.threshold(img, img.mean(), 255, cv2.THRESH_BINARY)
    img = img.reshape(1,28,28,1)

    predicao = modelo.predict(img)
    print(predicao)
    resultado = np.argmax(predicao)
    print(resultado)
    os.remove(filepath)
    
    return render_template("resultado.html", resultado=resultado)

if __name__ == "__main__":
    if not os.path.exists('temp'):
        os.makedirs('temp')
    app.run(host='localhost', port=5000)