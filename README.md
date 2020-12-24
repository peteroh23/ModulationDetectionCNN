# ModulationDetectionCNN
ECE 5760 Advanced Microcontroller, Final Project: Radio Modulation Detecter that utilizes a CNN built on DE1-SoC (FPGA)


We created a radio modulation classifier that predicts the modulation scheme of received wireless signals with a Convolutional Neural Network implemented on the DE1-SoC. We utilized a Software-Defined Radio (an RTL-SDR) –– attached to the ARM processor via USB –– in order to obtain local radio signals. The radio signals are then sent over to the FPGA for classification by a CNN (AM-SSB, WBFM, GFSK). In addition, a spectrogram of the Fast Walsh-Hadamard Transform of the signal overtime was plotted on a VGA screen to visualize the received signals.
