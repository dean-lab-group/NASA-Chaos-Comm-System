BackEndDecoder

New plan is to have ADC sample at high enough rate to see individual peaks of matched filter output.
Then use an "integrator" to smooth rapid square wave into a single high value.
Use this processed signal then to crunch through the binary data to determine actual values beign sent.




1) Change FrontEnd signal-start sequence to 1-1-0-0