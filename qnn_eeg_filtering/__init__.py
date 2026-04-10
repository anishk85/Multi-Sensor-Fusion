"""
Quantum Neural Network (RQNN) EEG/Signal Filtering Package.

Implements the Recurrent Quantum Neural Network from:
Gandhi, V., Prasad, G., Coyle, D., Behera, L., & McGinnity, T. M. (2014).
"Quantum Neural Network-Based EEG Filtering for a Brain-Computer Interface."
IEEE Transactions on Neural Networks and Learning Systems, 25(2), 278-288.
"""

from .rqnn_model import RQNNFilter
from .pso_optimizer import PSOOptimizer
