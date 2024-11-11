#!/usr/bin/env python

import numpy as np
import os
from sklearn.model_selection import train_test_split
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import LSTM, Dense
from tensorflow.keras.callbacks import TensorBoard

class IntentionCore:
    def __init__(self):
        self.current_file_path = os.path.dirname(os.path.abspath(__file__))
        self.DATA_PATH = os.path.join(self.current_file_path, 'MP_Data') 
        # self.actions = np.array(['grab', 'push', 'turn'])
        self.actions = np.array(['grab', 'push'])
        # self.actions = np.array(['push'])
        self.no_sequences = 30
        self.sequence_length = 30

        self.model = Sequential()
        self.model.add(LSTM(64, return_sequences=True, activation='relu', input_shape=(29,165)))
        self.model.add(LSTM(128, return_sequences=True, activation='relu'))
        self.model.add(LSTM(64, return_sequences=False, activation='relu'))
        self.model.add(Dense(64, activation='relu'))
        self.model.add(Dense(32, activation='relu'))
        self.model.add(Dense(self.actions.shape[0], activation='softmax'))

        self.model.load_weights(os.path.join(self.current_file_path, 'action.h5'))
        self.sequence = []
        print("haha")
    def folder_creation(self):
        for action in self.actions: 
            for sequence in range(self.no_sequences):
                try: 
                    os.makedirs(os.path.join(self.DATA_PATH, action, str(sequence)))
                except:
                    pass

    def add_trainning_data(self, _np_array, action, sequence, frame_num):       
        npy_path = os.path.join(self.DATA_PATH, action, str(sequence), str(frame_num))
        np.save(npy_path, _np_array)


    def train_model(self):
        label_map = {label:num for num, label in enumerate(self.actions)}

        sequences, labels = [], []
        for action in self.actions:
            for sequence in range(self.no_sequences):
                window = []
                for frame_num in range(1, self.sequence_length):
                    res = np.load(os.path.join(self.DATA_PATH, action, str(sequence), "{}.npy".format(frame_num)))
                    window.append(res)
                sequences.append(window)
                labels.append(label_map[action])

        print(np.array(sequences).shape)
        print(np.array(labels).shape)

        X = np.array(sequences)
        y = to_categorical(labels).astype(int)
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.05)

        # print(y_test.shape)

        log_dir = os.path.join('Logs')
        tb_callback = TensorBoard(log_dir=log_dir)

        res = [.7, 0.2, 0.1]
        self.actions[np.argmax(res)]
        self.model.compile(optimizer='Adam', loss='categorical_crossentropy', metrics=['categorical_accuracy'])
        self.model.fit(X_train, y_train, epochs=2000, callbacks=[tb_callback])
        self.model.summary()

        self.model.save('action.h5')

        # print("=========================================")
        # res = self.model.predict(X_test)
        # print(self.actions[np.argmax(res[4])])
        # print(self.actions[np.argmax(y_test[4])])

    def live_prediction(self, _data):
        # os.path.join(self.current_file_path, 'action.h5')
        # self.model.load_weights('action.h5')
        

        self.sequence.append(_data)
        self.sequence = self.sequence[-29:]
        
        if len(self.sequence) == 29:
            res = self.model.predict(np.expand_dims(self.sequence, axis=0))[0]
            print(self.actions[np.argmax(res)])
            return self.actions[np.argmax(res)]
        
        return ""
                


if __name__ == '__main__':
    intent_core = IntentionCore()
    # intent_core.skeleton_view()
    # intent_core.folder_creation()
    # intent_core.training_view()
    # intent_core.train_model()
    # intent_core.live_prediction()