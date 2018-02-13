# TisLib for AMI TCG2 Module.

Background of this work.
AMI TCG2 module does not support measure boot on the TPM chip which is using I2C. This work is to overcome such limitation by leveraging the TIS driver in Tiano framework to AMI Aptio5 TCG2 module.

Description of the work.
* Add TIS(TPM interface specification) driver to relay TPM 1.2 commands to Infineon 9645 I2C TPM 1.2 part.
* I2C bus is contributed from Intel BayTrial SOC. 
* Measure boot tested ok on Adlink cExpress-BT board with I2C Infineon 9645 TPM 1.2 chip.

