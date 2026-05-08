from .cnn import CNN
from .cnn_lstm import CNNLSTM
from .cnn_node import CNNNode

MODELS = {
    'cnn':      CNN,
    'cnn_lstm': CNNLSTM,
    'cnn_node': CNNNode,
}

# seq_len each model expects from SteeringDataset.
SEQ_LEN = {
    'cnn':      1,
    'cnn_lstm': 3,
    'cnn_node': 3,
}
