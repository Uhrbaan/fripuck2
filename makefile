SCHEMA_DIR = protocol-schemas
SCHEMAS = $(wildcard $(SCHEMA_DIR)/*.fbs)

generate: 
	mkdir -p firmware/shared/flatcc-generated
	mkdir -p api/go/internal
	mkdir -p api/python/src/fripuck2/_generated
	
	flatcc -a -I $(SCHEMA_DIR) -o firmware/shared/flatcc-generated $(SCHEMAS)
	flatc --go --gen-object-api -I $(SCHEMA_DIR) -o api/go/internal $(SCHEMAS)
	flatc --python --gen-object-api --gen-onefile -I $(SCHEMA_DIR) -o api/python/src/fripuck2/_generated $(SCHEMAS)

clean: 
	rm -rf api/go/internal/FripuckProtocol
	rm -rf api/python/src/fripuck2/_generated/FripuckProtocol
	rm -rf firmware/shared/flatcc-generated/*.h

flash: 
	pio run -t upload --project-dir firmware/controller-freertos
	pio run -t upload --project-dir firmware/radio