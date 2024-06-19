from flask import Flask, request, jsonify, render_template
import cv2
import numpy as np
from tensorflow.keras.models import load_model
modelo = load_model('modelo_mnist.h5')

app = Flask(__name__)

app.config["TEMPLATES_AUTO_RELOAD"] = True

@app.route('/', methods=['GET'])
def home():
    return render_template('index.html')

@app.route('/echo', methods=['GET'])
def echo():
    return jsonify({'result': 'success'})

@app.route('/modelo', methods=['POST'])
def model():
    print(request.files)
    file = request.files['teste_modelo']
    # file.save(file.filename)
    file.save(file.filename)
    img = cv2.imread(file.filename)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) 
    img = cv2.resize(img, (28,28))
    img = img / img.max()
    _, img = cv2.threshold(img, img.mean(), 255, cv2.THRESH_BINARY)
    img = img.reshape(1,28,28,1)
    predicao = modelo.predict(img)
    print(predicao)
    np.argmax(predicao)
    return jsonify({'result': str(np.argmax(predicao))})
if __name__ == "__main__":
    app.run(host='localhost', port=5000)