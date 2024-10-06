from .mechlmm_core import MechLMMCore

from flask import Flask, request, jsonify, render_template

from dotenv import load_dotenv
load_dotenv()

app = Flask(__name__)

mechlmm_core = MechLMMCore()

data_store = {}

@app.route('/')
def root():
    return render_template('index.html')
    # help_text = """mechlmm server is up 
    #     use GET on /mechlmm to get avaliable API for mechlmm
    #     """
    
    # return help_text

@app.route('/mechlmm/', methods=['GET'])
def mechlmm_api():
    return jsonify({'api_type': '/mechlmm/chat/text, chat_img, chat_video, chat_kowledge'})

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

    result = mechlmm_core.chat_tool(tools, question, base_img, schema, tag)

    return jsonify(result)

@app.route('/mechlmm/chat/text', methods=['POST'])
def chat_text():
    data = request.json

    if not data or 'question' not in data:
        return jsonify({'error': 'Invalid Data'}), 400
    
    question = data['question']
    schema = data.get('schema', None)
    tag = data.get('tag', None)

    result = mechlmm_core.chat_text(question, schema, tag)

    return jsonify(result)

@app.route('/mechlmm/chat/image', methods=['POST'])
def chat_img():
    data = request.json

    if not data or 'question' not in data:
        return jsonify({'error': 'Invalid Data'}), 400
    
    question = data['question']
    schema = data.get('schema', None)
    tag = data.get('tag', None)
    base_img = data.get('base_img', None)

    result = mechlmm_core.chat_img(question, base_img, schema, tag)

    return jsonify(result)

@app.route('/mechlmm/chat/tool', methods=['POST'])
def chat_tool():
    data = request.json
    
    if not data or 'question' not in data:
        return jsonify({'error': 'Invalid Data'}), 400
    
    question = data['question']
    schema = data.get('schema', None)
    tag = data.get('tag', None)
    base_img = data.get('base_img', None)
    tools = data.get('tools', None)

    result = mechlmm_core.chat_tool(tools, question, base_img, schema, tag)

    return jsonify(result)

@app.route('/mechlmm/chat/data', methods=['POST'])
def chat_data():
    data = request.json
    
    if not data or 'question' not in data:
        return jsonify({'error': 'Invalid Data'}), 400
    
    question = data['question']
    schema = data.get('schema', None)
    tag = data.get('tag', None)

    result = mechlmm_core.chat_data(question, schema, tag)

    return jsonify(result)

def main():
    app.run(host='0.0.0.0', port=5001)

if __name__ == '__main__':
    main()
