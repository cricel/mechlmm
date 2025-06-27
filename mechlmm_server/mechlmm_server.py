from mechlmm_core import MechLMMCore
from mechlmm_py import DB_Core

from flask import Flask, request, jsonify, render_template

from dotenv import load_dotenv
load_dotenv()

app = Flask(__name__)

mechlmm_core = MechLMMCore()
db_core = DB_Core(False, "localhost")

@app.route('/')
def root():
    return render_template('index.html')

@app.route('/mechlmm/', methods=['GET'])
def mechlmm_api():
    return jsonify({'content': 'welcome to mechlmm'})

@app.route('/mechlmm/chat', methods=['POST'])
def chat():
    data = request.json
    
    if not data or 'question' not in data:
        return jsonify({'error': 'Invalid Data'}), 400
    
    question = data['question']
    schema = data.get('schema', None)
    tag = data.get('tag', None)
    base_img = data.get('base_img', None)
    tools = data.get('tools', None)
    model = data.get('model', None)

    print(f"question: {question}")
    print(f"image: {base_img}")
    result, return_tag, result_type = mechlmm_core.chat(question, tools, base_img, schema, tag, model)

    return jsonify({"result": result, 
                    "tag": return_tag,
                    "type": result_type
                    })

@app.route('/mechlmm/chat/qa', methods=['POST'])
def chat_qa():
    data = request.json
    
    if not data or 'question' not in data:
        return jsonify({'error': 'Invalid Data'}), 400
    
    question = data['question']

    result = mechlmm_core.chat_knowledge(question)

    return jsonify({"result": result})

@app.route('/mechlmm/chat/data', methods=['POST'])
def chat_data():
    data = request.json
    
    if not data or 'question' not in data:
        return jsonify({'error': 'Invalid Data'}), 400
    
    question = data['question']

    result = mechlmm_core.chat_datalog(question)

    return jsonify({"result": result})

@app.route('/database/get_table/<table_name>', methods=['GET'])
def get_table_data(table_name):
    result = db_core.get_table(table_name)

    return jsonify(result)

def main():
    app.run(host='0.0.0.0', port=5001)

if __name__ == '__main__':
    main()
