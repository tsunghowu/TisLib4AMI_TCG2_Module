# TisLib to extend Tis support in AMI UEFI AptioV TCG2 Module.

Background of this work.
* AMI AptioV TCG2 module does not support TPM chip, which is using I2C bus. This work is to overcome such limitation by leveraging the TIS driver in Tiano framework to AMI Aptio5 TCG2 module and replace its original TIS driver for LPC TPM 1.2 chips.

Description of the work.
* Add TIS(TPM interface specification) driver to relay TPM 1.2 commands to Infineon 9645 I2C TPM 1.2 part.
* I2C bus is contributed from Intel BayTrial SOC. 
* Measure boot tested ok on evaluation board(Minnowboard max) with I2C Infineon 9645 TPM 1.2 chip.

Future work.
* To extend the work to support Infineon SLB 9665 TPM 2.0 chip.
