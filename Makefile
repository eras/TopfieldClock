PACKS:=-package unix,batteries,core -thread

%: %.ml
	ocamlfind ocamlc -dtypes -linkpkg $(PACKS) -o $@ $<

send-sequence: send-sequence.ml
