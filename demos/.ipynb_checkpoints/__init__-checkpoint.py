from .crosscorrelation import CrossCorrelationDemo
from .convolution import ConvolutionDemo
from .signal_analyzer import SignalAnalyzer
from .sinewave import SineWaveDemo
from .phasor import PhasorDemo
from .vector_sum import VectorSumDemo
from .frequency_mixing import FrequencyMixingDemo, FrequencyMixingSpectrumDemo
from .aliasing import AliasingDemo, ComplexAlias, AliasingDemo2
from .dft import DFT_Demo
from .sinusoid_spectrum import SinusoidSpectrumDemo
from .spectral_leakage import SpectralLeakageDemo
from .dtft import DTFT_Demo
from .frequency_response import FreqRespDemo
from .euler_demo import EulerDemo1D
from .z_transform import tfPlot, pzPlot, Magnitude_dB, visualizeTF, showOscillation, showDiscreteOscillation, HsPlot
from .Frekvensrespons import ContFreqRespDemo

__all__ = ["CrossCorrelationDemo",
           "ConvolutionDemo",
           "SignalAnalyzer",
           "SineWaveDemo",
           "PhasorDemo",
           "VectorSumDemo",
           "FrequencyMixingDemo",
           "FrequencyMixingSpectrumDemo",
           "AliasingDemo",
           "ComplexAlias",
           "AliasingDemo2",
           "DFT_Demo",
           "SinusoidSpectrumDemo",
           "SpectralLeakageDemo",
           "DTFT_Demo",
           "FreqRespDemo",
           "EulerDemo1D",
           "tfPlot",
           "pzPlot",
           "Magnitude_dB", 
           "visualizeTF", 
           "showOscillation", 
           "showDiscreteOscillation",
           "HsPlot",
           "ContFreqRespDemo"]