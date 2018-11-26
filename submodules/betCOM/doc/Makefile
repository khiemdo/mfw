include ../betcom.mk

all: html markdown

html: $(PROTO_FILES)
	protoc -I$(BETCOM_DIR) --doc_out=html,betcom.html:. $(PROTO_FILES)


markdown: $(PROTO_FILES)
		protoc -I$(BETCOM_DIR) --doc_out=markdown,betcom.md:. $(PROTO_FILES)
