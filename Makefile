BINDING_DIR := ./brml

gen:
	c-for-go -out ./ $(BINDING_DIR)/brml.yml
	cd $(BINDING_DIR); \
	go tool cgo -godefs types.go > types_gen.go; \
	rm -rf types.go _obj
