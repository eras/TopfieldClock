open Batteries

(* 00100000 01110101 01100111 0S$1      111110S$2      111 10S$3      1111110 S$4       11000111 2 011000000 2 *)

(* 00100000 01110101 01100111 0XXXXXXX 111110XX XXXXX111 10XXXXXX X1111110 XXXXXXX1 10001111 11111110 11000000 11111111 *)
(* 00100000 01110101 01100111 00000000 11111000 00000111 10000000 01111110 00000001 10001111 11111110 11000000 11111111 *)

(* 00100000 01110101 01100111 00001001 11111001 00100111 10011000 01111110 00010011 10001111 11111110 11000000 11111111 *)

let show i = Printf.printf "%d\n%!" i

let main () =
  let sequence = Std.input_all Pervasives.stdin in
  let rec loop at accu bit =
    if at >= String.length sequence 
    then 
      if bit <> 0
      then show accu
      else ()
    else 
      match String.get sequence at with
	| '0' -> advance_bit at accu bit
	| '1' -> advance_bit at (accu lor (1 lsl bit)) bit
	| _ -> loop (at + 1) accu bit
  and advance_bit at accu bit =
    let (at, bit, accu) = 
      if bit + 1 = 8 
      then begin
	show accu; 
	(at + 1, 0, 0) 
      end else (at, bit + 1, accu) in
      loop (at + 1) accu bit
  in
    loop 0 0 0

let _ = main ()
