MAKEFSDATA = ../../../tools/makefsdata/makefsdata "-svr:Maven (https://www.maverick-embedded.co.uk/)"
SRCS	:= app_fsdata.c assets_fsdata.c provision_fsdata.c

all:	$(SRCS)

clean:
	rm -f $(SRCS)

app_fsdata.c:
	$(MAKEFSDATA) -f:$@ -defl app

assets_fsdata.c:
	$(MAKEFSDATA) -f:$@ -defl assets

provision_fsdata.c:
	$(MAKEFSDATA) -f:$@ -r -defl provision

#	$(MAKEFSDATA) -f:$@ -ssi:provision.ssi -r provision

