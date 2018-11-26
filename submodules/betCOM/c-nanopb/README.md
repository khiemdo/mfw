# BetCALL nanopb example
## Preparation
Make sure you checked out the nanopb repository from our [gitlab](http://gitlab.syscop.de/highwind/nanobp). To find the dependencies the nanopb repo has to be on the same level as the betCOM repo. 
```
git clone http://gitlab.syscop.de/highwind/nanobp.git
```

## Create Protobuf Python files
The `-I` folder should point to the folder where the `.proto` files are saved.
Since nanopb focuses on usability of protobuf on embedded devices we can define
the maximum count of dynamic fields like `repeated` in a separate option file in the
options folder of the repository or even local. Nanobp is called as a plug-in of protoc.
Using the make-rules from nanopb and betCOM we end up with a compilation command like:
```
	$(PROTOC) $(PROTOC_OPTS) -I$(BETCOM_DIR) --nanopb_out="-v -I $(BETCOM_OPTIONS_DIR):./" $(PROTO_FILES)
```

## Run example
Run `make` to start the generation of the protobuf based code.


In the same folder run following command:
```
./test_betcall
```
The program shows how to use the defined Sensors datatype of BetCALL.
The program gets the current system time. Writes it to the Message.
The message is then packed and unpacked.
If everything works fields have the same values before and after packing.


## Usage
Make sure that you always refer to the `.proto` of the betCOM repository.
Otherwise it is not guaranteed that the files stay up-to-date.
Therefore it is hardly recommended to use the makefile-rules defined `betcom.mk`
