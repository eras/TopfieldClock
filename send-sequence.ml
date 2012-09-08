open Batteries

let open_serial tty bps =
  let fd = Unix.openfile tty [Unix.O_RDWR; Unix.O_NONBLOCK] 0 in
  let tio = Unix.tcgetattr fd in
  let tio = {
    tio with
      Unix.c_clocal	= true;
      c_obaud		= bps;
      c_ibaud		= bps;
      c_csize		= 8;
      c_cstopb		= 1;
      c_inlcr           = false;
      c_icrnl		= false;
      c_opost           = false;
      c_isig		= false;
      c_icanon		= false;
      c_echo		= false;
      c_vtime		= 1;
      c_vmin		= 1;
  } in
    Unix.tcsetattr fd Unix.TCSANOW tio;
    Unix.clear_nonblock fd;
    fd

let write_all fd str = 
  for c = 0 to String.length str - 1 do
    ignore (Unix.select [] [] [] 0.001);
    ignore (Unix.write fd str c 1)
  done

type op = 
  | Bool of bool
  | DelayHigh
  | Sequence of char

let ops_of_flags flags =
  let bit_offsets = [
    0, `A;
    5, `B;
    6, `C;
    1, `D;
    4, `E;
    3, `F;
    2, `G;
    7, `Dot;
  ] in
    List.init 7 **> fun idx ->
      if List.mem (List.assoc idx bit_offsets) flags
      then Bool false
      else Bool true

let sequences = 
  ['0', [`A; `B;     `D; `E; `F; `G];
   '1', [            `D;         `G]; (* ?? *)
   '2', [`A;     `C; `D; `E; `F;   ];
   '3', [`A;     `C; `D;     `F; `G];
   '4', [    `B; `C; `D;         `G];
   '5', [`A; `B; `C;         `F; `G];
   '6', [`A; `B; `C;     `E; `F; `G];
   '7', [`A;         `D;         `G]; (* ?? *)
   '8', [`A; `B; `C; `D; `E; `F; `G];
   '9', [`A; `B; `C; `D;     `F; `G];
   ' ', [                          ];
  ]

(*

- : string = "1?00111"
# binstr_of_bool (Sequence '1');;
- : string = "1100111"
# binstr_of_bool (Sequence '7');;
- : string = "1000111"


- : string = "???????"
# binstr_of_bool (Sequence '0');;
- : string = "1000000"
# binstr_of_bool (Sequence '2');;
- : string = "0001001"
# binstr_of_bool (Sequence '3');;
- : string = "0000011"
# binstr_of_bool (Sequence '4');;
- : string = "0100110"
# binstr_of_bool (Sequence '5');;
- : string = "0010010"
# binstr_of_bool (Sequence '6');;
- : string = "0010000"
# binstr_of_bool (Sequence '8');;
- : string = "0000000"
# binstr_of_bool (Sequence '9');;
- : string = "0000010"
# 

*)


let rec binstr_of_bool = function
  | Bool false -> "0"
  | Bool true -> "1"
  | DelayHigh -> String.make 8 '1'
  | Sequence seq -> 
      try String.concat "" (List.map binstr_of_bool (ops_of_flags (List.assoc seq sequences)))
      with Not_found -> Printf.ksprintf failwith "unknown sequence %c" seq

let bool_of_binstr = function 
  | '0' -> Some (Bool false)
  | '1' -> Some (Bool true)
  | '2' -> Some DelayHigh
  | _ -> None

let not1 f x = not (f x)

let split_by p xs =
  (List.take_while (not1 p) xs, 
   match List.drop_while (not1 p) xs with
     | [] -> []
     | _::xs -> xs)

let rec parts_by p xs =
  match xs with
    | [] -> []
    | _ -> 
	let (left, right) = split_by p xs in
	  left::parts_by p right

let send_sequence fd seq = 
  let seq = List.of_enum seq in
    ( try
	write_all fd "b";
	write_all fd (String.concat "" (List.map binstr_of_bool seq));
	write_all fd ";";
      with Sys.Break ->
	write_all fd ";" );
    let buf = String.make 1 ' ' in
      ignore (Unix.read fd buf 0 1);
      if String.get buf 0 <> '.' then
	Printf.eprintf "Failed to send sequence"
	(* ignore (Core.Core_unix.nanosleep 0.0000520); *)


let sequence_of_str str =
  let rec aux = function
    | [] -> []
    | 'S'::ch::rest -> Sequence ch::aux rest
    | ch::rest -> 
	match bool_of_binstr ch with
	  | None -> aux rest
	  | Some bool -> bool::aux rest
  in
    List.enum (aux (String.explode str))

let do_send serial_port str =
  let fd = open_serial serial_port 9600 in
    send_sequence fd (sequence_of_str str);
    Unix.close fd

let main () =
  Sys.catch_break true;
  let args = List.tl (Array.to_list Sys.argv) in
    match args with
      | serial_port::sequence::[] ->
	  do_send serial_port sequence
      | serial_port::[] ->
	  do_send serial_port (Std.input_all Pervasives.stdin)
      | _ -> 
	  Printf.printf "usage: send-sequence /dev/ttyUSB1 001010100012\n";
	  Printf.printf "Unsupported characters are ignored in the sequence\n"

let _ = main ()
