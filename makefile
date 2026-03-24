SCHEMAS = telemetry.fbs commands.fbs

generate: 
	flatcc -a -o firmware/shared/flatcc-generated $(SCHEMAS)
	flatc --go --gen-object-api -o api/go/internal $(SCHEMAS)
	flatc --python --gen-object-api -o api/python/src/fripuck2/_generated $(SCHEMAS)

clean: 
	rm -rf api/go/internal/FripuckProtocol
	rm -rf api/python/src/fripuck2/_generated/FripuckProtocol
	rm -rf firmware/shared/flatcc-generated/*.h
