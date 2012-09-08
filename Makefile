PACKS:=-package unix,batteries,core -thread

%: %.ml
	ocamlfind ocamlc -g -dtypes -linkpkg $(PACKS) -o $@ $<

convert-sequence: convert-sequence.ml

send-sequence: send-sequence.ml

difference: difference.ml
