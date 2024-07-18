module VFPUnit(
    input wire [159:0] Va, Vb,
    input wire [1:0] FPControl,
    output wire [159:0] VResult,
    output wire [19:0] ALUFlags
);

    wire [31:0] a_1, a_2, a_3, a_4, a_5;
    wire [31:0] b_1, b_2, b_3, b_4, b_5;
    wire [31:0] Result_1, Result_2, Result_3, Result_4, Result_5;
    wire [3:0] ALUFlags_1, ALUFlags_2, ALUFlags_3, ALUFlags_4, ALUFlags_5;

    assign {a_1, a_2, a_3, a_4, a_5} = {Va[31:0], Va[63:32], Va[95:64], Va[127:96], Va[159:128]};
    assign {b_1, b_2, b_3, b_4, b_5} = {Vb[31:0], Vb[63:32], Vb[95:64], Vb[127:96], Vb[159:128]};

    FPUnit u1 (.a(a_1), .b(b_1), .FPControl(FPControl), .Result(Result_1), .ALUFlags(ALUFlags_1));
    FPUnit u2 (.a(a_2), .b(b_2), .FPControl(FPControl), .Result(Result_2), .ALUFlags(ALUFlags_2));
    FPUnit u3 (.a(a_3), .b(b_3), .FPControl(FPControl), .Result(Result_3), .ALUFlags(ALUFlags_3));
    FPUnit u4 (.a(a_4), .b(b_4), .FPControl(FPControl), .Result(Result_4), .ALUFlags(ALUFlags_4));
    FPUnit u5 (.a(a_5), .b(b_5), .FPControl(FPControl), .Result(Result_5), .ALUFlags(ALUFlags_5));

    assign VResult = {Result_5, Result_4, Result_3, Result_2, Result_1};
    assign ALUFlags = {ALUFlags_5, ALUFlags_4, ALUFlags_3, ALUFlags_2, ALUFlags_1};
endmodule



module FPUnit(
    input [31:0] a, b,
    input [1:0] FPControl,
    output reg [31:0] Result,
    output reg [3:0] ALUFlags
  );

  wire neg, zero, carry, overflow;
  wire [31:0] faddResult32;//FADD RESULT 32
  wire [15:0] faddResult16;//FADD RESULT 16
  wire [31:0] fmulResult32;// FMULT RESUL 32
  wire [15:0] fmulResult16;// FMULT RESUL 32

  //Instanciaci�n de los flags
  wire [3:0] ALUFlagsAdd32; //Alu flags 32 ADD
  wire [3:0] ALUFlagsAdd16; //Alu flags 16 ADD
  wire [3:0] ALUFlagsMul32; //Alu flags 32 MUL
  wire [3:0] ALUFlagsMul16; //Alu flags 16 MUL


  fadd fadd_instance32(
         .a(a),
         .b(b),
         .Result(faddResult32),
         .ALUFlags(ALUFlagsAdd32)
       );
  fadd16 fadd_instance16(
           .a(a[15:0]),//Solo le doy 16 bits del menos al mas significativ
           .b(b[15:0]),
           .Result(faddResult16),
           .ALUFlags(ALUFlagsAdd16)
         );

  fpmul32 fmul_instance32(
            .a(a),
            .b(b),
            .Result(fmulResult32),
            .ALUFlags(ALUFlagsMul32)
          );

  fpmul16 fmul_instance16(
      .a(a[15:0]),
      .b(b[15:0]),
      .Result(fmulResult16),
      .ALUFlags(ALUFlagsMul16)
  );

  always @(*)
  begin
    case (FPControl)
      2'b00:
      begin
        Result = faddResult32;
        ALUFlags = ALUFlagsAdd32;
      end

      2'b01:
      begin
        Result = faddResult16;
        ALUFlags = ALUFlagsAdd16;
      end

      2'b11:
      begin
        Result = fmulResult16;
        ALUFlags = ALUFlagsMul16;
      end
      2'b10:
      begin
        Result = fmulResult32;
        ALUFlags = ALUFlagsMul32;
      end
    endcase
  end



endmodule



module sub( m1,m2,q,em, ee, e1,e2, m_R, round, exp_R);//MODULE SUB
  input [22:0]m1;//mantisa sin bit implicito
  input [22:0]m2; //mantisa sin bit implicito

  input [7:0] q;//La cantidad q debo shiftear
  input [7:0] e1;
  input [7:0] e2;
  output [7:0] exp_R;
  input em; //Si el exponente de m1 es mayor que el de m2
  input ee; //Si los exponentes son iguales
  output [24:0]m_R;
  output round;

  //A�ado el bit implicito
  wire [255:0]m1b; //Para mejorar la precision m1b y m2b deberia ser de aprox tama�o 255
  wire [255:0]m2b;


  //-------
  assign m1b[254:232] = m1[22:0];
  assign m1b[231:0] = 232'b0;
  assign m2b[231:0] = 232'b0;
  assign m2b[254:232] = m2[22:0];
  assign m1b[255]=1'b1;
  assign m2b[255]=1'b1;
  //Ya tengo las mantisas bonitas

  //Ahora el shifteo:
  reg [255:0]shifted_m1b;
  reg [255:0]shifted_m2b;

  always @*
  begin
    if (em)
    begin
      shifted_m2b = (q > 0) ? (m2b >> q) : m2b;//Arreglando
      shifted_m1b = m1b; // m1 ya est� en la posici�n correcta
    end
    else
    begin
      // Desplazar m1 a la posici�n de m2 (|q| posiciones a la izquierda si q < 0)
      shifted_m1b = (q > 0) ? (m1b >> q) : m1b; //Siempre hago shifteo del menor al mayor
      shifted_m2b = m2b; // m2 ya est� en la posici�n correcta

    end
  end
  //Ahora por fin si puedo restar pipi
  reg [255:0]resta;

  always @*
  begin
    if (em)
    begin //Para no hacer el swap
      resta = shifted_m1b - shifted_m2b;
    end
    else
    begin
      if (ee)
        resta = shifted_m1b - shifted_m2b;
      else // Cuando la otra mantisa es mas grandecita
        resta = shifted_m2b - shifted_m1b;
    end
  end

  //Logica para el shifteo o normalizacion:
  reg [7:0]toshift;
  //AQUI toshit debe tener el valor del cuanto es necesario
  //el dhifteo en resta para llegar a 1, por ejemplo si es
  //0001, to shift debe ser 0000011
  integer i;
  reg found_one;

  always @*
  begin
    toshift = 0;
    found_one = 0;
    for (i = 255; i >= 0; i = i - 1)
    begin
      if (!found_one && resta[i] == 1'b1)
      begin
        toshift = 255 - i;
        found_one = 1;
      end
    end
  end
  //Hasta aqui ya tenemos toshift

  //assign m_R[24] = 1'b0;  //Aqui le doy 25 bits para no ajsutar la logica de los condicionales [0] no hay overflow virtual
  assign m_R[24:0] = {resta[255:231]<<toshift}; //Aqui le doy 25 bits para no ajsutar la logica de los condicionales
  //Necesito logica para el redondeo por eso
  assign round = resta[229]; //Si el 5 bit es 1 se redondea:

  //Calculo del nuevo exponente:
  reg [7:0] exp_R_reg;

  always @*
  begin
    if (m_R[24:0] == 25'b0 && toshift == 8'b0)
    begin
      exp_R_reg = 8'b0; // Si m_R es todo 0 y toshift es todo 0, exp_R se establece en 0.
    end
    else
    begin
      exp_R_reg = em ? (e1 - toshift) : (e2 - toshift);
    end
  end

  assign exp_R = exp_R_reg;






endmodule

module fadd(//MODULE ADD32
    input [31:0] a, b,
    output reg [31:0] Result,
    output reg [3:0] ALUFlags
  );
  wire s1, s2;
  wire [7:0] e1, e2;
  wire [23:0] m1, m2;
  //Asignaci�n de las partes:
  //Signos
  assign s1 = a[31];
  assign s2 = b[31];
  //Exponentes
  assign e1 = a[30:23];
  assign e2 = b[30:23];
  //Mantisas
  assign m1[22:0] = a[22:0];
  assign m2[22:0] = b[22:0];

  //1) El exponente mayor
  wire em; //Es 1 si e1 es mayor caso contrario toma 0
  assign em = (e1 > e2);
  wire ee; //Expos iguales
  assign ee = (e1 == e2);
  wire se;// Signos iguales
  assign se = (s1 == s2);


  // Calculo el valor que debo shitear
  reg [7:0] q; //valor de shifteo
  always @*
  begin
    if (em)
    begin
      // Calculo q = e1 - e2
      q = e1 - e2;
    end
    else
    begin
      // Calculo q = e2 - e1
      q = e2 - e1;
    end
  end

  //A�ado el bit implicito a las mantisas
  wire b1;
  assign b1 =1;
  assign m1[23] = b1;
  assign m2[23] = b1;
  //Uso el valor de q para shiftear el numero de posicions a la mantisa con el exponente menor
  wire round; //---IMPORTANTEEE--- o el redondeo positivo

  reg [23:0] shifted_m1, shifted_m2;
  reg [24:0] mantisa_for_round; //Mantisa q nos permite ver el redondeo
  always @*
  begin
    if (em)
    begin
      // Desplazar m2 a la posici�n de m1 (q posiciones a la izquierda si q > 0)
      shifted_m2 = (q > 0) ? (m2 >> q) : m2;//Arreglando
      shifted_m1 = m1; // m1 ya est� en la posici�n correcta
      mantisa_for_round = (q > 0) ? (m2 >> q) : m2;
    end
    else
    begin
      // Desplazar m1 a la posici�n de m2 (|q| posiciones a la izquierda si q < 0)
      shifted_m1 = (q > 0) ? (m1 >> q) : m1; //Siempre hago shifteo del menor al mayor
      shifted_m2 = m2; // m2 ya est� en la posici�n correcta
      mantisa_for_round = (q > 0) ? (m1 >> q) : m1;

    end
  end
  wire [24:0]m_R;
  wire [7:0]exp_R;
  wire roundn;//Para ver si se usa el redondeo negativo
  sub subtuki( .m1(m1),.m2(m2),.q(q),.em(em),.ee(ee),.e1(e1),.e2(e2), .m_R(m_R), .round(roundn), .exp_R(exp_R));

  //BIT PARA REDONDEO:
  assign round = (se?mantisa_for_round[0]:roundn);
  //Si son signos iguales redondeo de suma, sino el de resta



  //Sumar las mantisas adecuadamente
  reg [24:0] mantisaS; //Ahora es de 25 bits, para el caso de overflow de mantisas
  always @*
  begin
    if (se)
    begin
      mantisaS = shifted_m1 + shifted_m2;  // signos iguales
    end
    else
    begin
      mantisaS = m_R;  // signos diferentes
    end
  end

  //"Peque�o" bucle apra reducir la cantidad de ifs:
  reg [7:0]exp;

  always @*
  begin
    if (em) //e1>e2
      if (se)
        exp = e1;
      else // Caso de resta
        exp = exp_R;

    else //e1<=e2
    begin
      if (ee) //e1 == e2
        if (se)
          exp = e1+1;
        else // Caso de resta
          exp = exp_R;
      else //e1<e2
        if (se)
          exp = e2;
        else // Caso de resta
          exp = exp_R;
    end
  end



  //Aqui lo redondeo
  always @*
  begin
    if (em) //e1>e2
    begin
      if(!mantisaS[24])
        Result = {s1, exp, round ? {mantisaS[22:0]+1'b1} : { mantisaS[22:0]}};
      else
        Result = {s1, exp, round ? {mantisaS[23:1]+1'b1} : { mantisaS[23:1]}};
    end
    else //e1<=e2
    begin
      if (ee) //e1 == e2
      begin
        if(!mantisaS[24])
          Result = {s1, exp, round ? {mantisaS[22:0]+1'b1} : { mantisaS[22:0]}};

        else
          Result = {s1, exp, round ? {mantisaS[23:1]+1'b1} : { mantisaS[23:1]}};

      end
      else //e1<e2
      begin
        if(!mantisaS[24])
          Result = {s2, exp, round ? {mantisaS[22:0]+1'b1} : { mantisaS[22:0]}};
        else
          Result = {s2, exp, round ? {mantisaS[23:1]+1'b1}: { mantisaS[23:1]}};
      end


    end
  end

  //LOGICA DE FLAGS.
  always @*
  begin
    // Zero Flag
    ALUFlags[3] = (mantisaS == 0);

    // Negative Flag
    ALUFlags[2] = (mantisaS[24] == 1);

    // Carry Flag (puede no ser relevante en punto flotante, as� que lo dejamos en 0)
    ALUFlags[1] = 1'b0;

    // Overflow Flag
    ALUFlags[0] = ((mantisaS[24] == 1) && (se == 1) && (mantisaS[23:0] == 24'b01111111111111111111111)) ||
            ((mantisaS[24] == 0) && (se == 1) && (mantisaS[23:0] == 24'b10000000000000000000000)) ||
            ((mantisaS[24] == 0) && (se == 0) && (mantisaS[23:0] == 24'b01111111111111111111111));
  end

endmodule



module sub16(
    input [9:0] m1, //mantisa sin bit implicito
    input [9:0] m2, //mantisa sin bit implicito
    input [4:0] q, //La cantidad que debo shiftear
    input [4:0] e1,
    input [4:0] e2,
    output [4:0] exp_R,
    input em, //Si el exponente de m1 es mayor que el de m2
    input ee, //Si los exponentes son iguales
    output [11:0] m_R,
    output round
  );

  //A�ado el bit implicito
  wire [127:0] m1b; //Para mejorar la precision m1b y m2b deber�a ser de tama�o 127
  wire [127:0] m2b;

  assign m1b[126:116] = m1[9:0];
  assign m1b[115:0] = 116'b0;
  assign m2b[126:116] = m2[9:0];
  assign m2b[115:0] = 116'b0;
  assign m1b[127] = 1'b1;
  assign m2b[127] = 1'b1;

  //Ahora el shifteo:
  reg [127:0] shifted_m1b;
  reg [127:0] shifted_m2b;

  always @*
  begin
    if (em)
    begin
      shifted_m2b = (q > 0) ? (m2b >> q) : m2b;
      shifted_m1b = m1b; // m1 ya est� en la posici�n correcta
    end
    else
    begin
      shifted_m1b = (q > 0) ? (m1b >> q) : m1b;
      shifted_m2b = m2b; // m2 ya est� en la posici�n correcta
    end
  end

  //Ahora por fin se puede restar
  reg [127:0] resta;

  always @*
  begin
    if (em)
    begin
      resta = shifted_m1b - shifted_m2b;
    end
    else
    begin
      if (ee)
        resta = shifted_m1b - shifted_m2b;
      else // Cuando la otra mantisa es m�s grande
        resta = shifted_m2b - shifted_m1b;
    end
  end

  //Logica para el shifteo o normalizacion:
  reg [4:0] toshift;
  integer i;
  reg found_one;

  always @*
  begin
    toshift = 0;
    found_one = 0;
    for (i = 127; i >= 0; i = i - 1)
    begin
      if (!found_one && resta[i] == 1'b1)
      begin
        toshift = 127 - i;
        found_one = 1;
      end
    end
  end

  assign m_R[11:0] = {resta[127:116] << toshift};
  assign round = resta[115]; //Si el bit 5 es 1 se redondea

  //Calculo del nuevo exponente:
  reg [4:0] exp_R_reg;

  always @*
  begin
    if (m_R[11:0] == 12'b0 && toshift == 5'b0)
    begin
      exp_R_reg = 5'b0; // Si m_R es todo 0 y toshift es todo 0, exp_R se establece en 0.
    end
    else
    begin
      exp_R_reg = em ? (e1 - toshift) : (e2 - toshift);
    end
  end

  assign exp_R = exp_R_reg;

endmodule



//FLOTING POINT ADD 16 BITS:

module fadd16(
    input [15:0] a, b,
    output reg [15:0] Result,
    output reg [3:0] ALUFlags
  );
  wire s1, s2;
  wire [4:0] e1, e2;
  wire [10:0] m1, m2;

  //Asignaci�n de las partes:
  //Signos
  assign s1 = a[15];
  assign s2 = b[15];
  //Exponentes
  assign e1 = a[14:10];
  assign e2 = b[14:10];
  //Mantisas
  assign m1[9:0] = a[9:0];
  assign m2[9:0] = b[9:0];

  //1) El exponente mayor
  wire em; //Es 1 si e1 es mayor caso contrario toma 0
  assign em = (e1 > e2);
  wire ee; //Expos iguales
  assign ee = (e1 == e2);
  wire se; // Signos iguales
  assign se = (s1 == s2);

  // Calculo el valor que debo shitear
  reg [4:0] q; //valor de shifteo
  always @*
  begin
    if (em)
    begin
      // Calculo q = e1 - e2
      q = e1 - e2;
    end
    else
    begin
      // Calculo q = e2 - e1
      q = e2 - e1;
    end
  end

  //A�ado el bit implicito a las mantisas
  wire b1;
  assign b1 = 1;
  assign m1[10] = b1;
  assign m2[10] = b1;

  //Uso el valor de q para shiftear el numero de posicions a la mantisa con el exponente menor
  wire round; //---IMPORTANTEEE--- o el redondeo positivo

  reg [10:0] shifted_m1, shifted_m2;
  reg [11:0] mantisa_for_round; //Mantisa que nos permite ver el redondeo
  always @*
  begin
    if (em)
    begin
      // Desplazar m2 a la posici�n de m1 (q posiciones a la izquierda si q > 0)
      shifted_m2 = (q > 0) ? (m2 >> q) : m2;
      shifted_m1 = m1; // m1 ya est� en la posici�n correcta
      mantisa_for_round = (q > 0) ? (m2 >> q) : m2;
    end
    else
    begin
      // Desplazar m1 a la posici�n de m2 (|q| posiciones a la izquierda si q < 0)
      shifted_m1 = (q > 0) ? (m1 >> q) : m1;
      shifted_m2 = m2; // m2 ya est� en la posici�n correcta
      mantisa_for_round = (q > 0) ? (m1 >> q) : m1;
    end
  end

  wire [11:0] m_R;
  wire [4:0] exp_R;
  wire roundn; //Para ver si se usa el redondeo negativo
  sub16 subtuki( .m1(m1), .m2(m2), .q(q), .em(em), .ee(ee), .e1(e1), .e2(e2), .m_R(m_R), .round(roundn), .exp_R(exp_R));

  //BIT PARA REDONDEO:
  assign round = (se ? mantisa_for_round[0] : roundn);

  //Sumar las mantisas adecuadamente
  reg [11:0] mantisaS; //Ahora es de 12 bits, para el caso de overflow de mantisas
  always @*
  begin
    if (se)
    begin
      mantisaS = shifted_m1 + shifted_m2;  // signos iguales
    end
    else
    begin
      mantisaS = m_R;  // signos diferentes
    end
  end

  //"Peque�o" bucle para reducir la cantidad de ifs:
  reg [4:0] exp;

  always @*
  begin
    if (em) //e1>e2
      if (se)
        exp = e1;
      else // Caso de resta
        exp = exp_R;
    else //e1<=e2
    begin
      if (ee) //e1 == e2
        if (se)
          exp = e1 + 1;
        else // Caso de resta
          exp = exp_R;
      else //e1<e2
        if (se)
          exp = e2;
        else // Caso de resta
          exp = exp_R;
    end
  end

  //Aqui lo redondeo
  always @*
  begin
    if (em) //e1>e2
    begin
      if(!mantisaS[11])
        Result = {s1, exp, round ? {mantisaS[9:0] + 1'b1} : {mantisaS[9:0]}};
      else
        Result = {s1, exp, round ? {mantisaS[10:1] + 1'b1} : {mantisaS[10:1]}};
    end
    else //e1<=e2
    begin
      if (ee) //e1 == e2
      begin
        if(!mantisaS[11])
          Result = {s1, exp, round ? {mantisaS[9:0] + 1'b1} : {mantisaS[9:0]}};
        else
          Result = {s1, exp, round ? {mantisaS[10:1] + 1'b1} : {mantisaS[10:1]}};
      end
      else //e1<e2
      begin
        if(!mantisaS[11])
          Result = {s2, exp, round ? {mantisaS[9:0] + 1'b1} : {mantisaS[9:0]}};
        else
          Result = {s2, exp, round ? {mantisaS[10:1] + 1'b1} : {mantisaS[10:1]}};
      end
    end
  end

  //LOGICA DE FLAGS.
  always @*
  begin
    // Zero Flag
    ALUFlags[3] = (mantisaS == 0);

    // Negative Flag
    ALUFlags[2] = (mantisaS[11] == 1);

    // Carry Flag (puede no ser relevante en punto flotante, as� que lo dejamos en 0)
    ALUFlags[1] = 1'b0;

    // Overflow Flag
    ALUFlags[0] = ((mantisaS[11] == 1) && (se == 1) && (mantisaS[10:0] == 11'b01111111111)) ||
            ((mantisaS[11] == 0) && (se == 1) && (mantisaS[10:0] == 11'b10000000000)) ||
            ((mantisaS[11] == 0) && (se == 0) && (mantisaS[10:0] == 11'b01111111111));
  end

endmodule


module mantissa_multiplier32 (
    input [23:0] mant_a,
    input [23:0] mant_b,
    output [47:0] mant_result
  );
  assign mant_result = mant_a * mant_b;
endmodule

module exponent_adder32 (
    input [7:0] exp_a,
    input [7:0] exp_b,
    output [7:0] exp_result
  );
  assign exp_result = exp_a + exp_b - 8'd127;
endmodule

module sign_handler32 (
    input sign_a,
    input sign_b,
    output sign_result
  );
  assign sign_result = sign_a ^ sign_b;
endmodule

module normalizer32 (
    input [47:0] mant_result,
    input [7:0] exp_result,
    output [22:0] normalized_mant,
    output [7:0] normalized_exp
  );
  wire [47:0] shifted_mant;
  wire [7:0] adjusted_exp;

  assign shifted_mant = mant_result >> 1;
  assign adjusted_exp = exp_result + 1;

  assign normalized_mant = mant_result[47] ? shifted_mant[46:24] : mant_result[45:23];
  assign normalized_exp = mant_result[47] ? adjusted_exp : exp_result;
endmodule

module fpmul32 (
    input [31:0] a,
    input [31:0] b,
    output [31:0] Result,
    output [3:0] ALUFlags
  );
  wire [7:0] exp_a, exp_b, exp_result;
  wire [23:0] mant_a, mant_b;
  wire [47:0] mant_result;
  wire sign_a, sign_b, sign_result;
  wire [22:0] normalized_mant;
  wire [7:0] normalized_exp;

  assign sign_a = a[31];
  assign sign_b = b[31];
  assign exp_a = a[30:23];
  assign exp_b = b[30:23];
  assign mant_a = {1'b1, a[22:0]};
  assign mant_b = {1'b1, b[22:0]};

  mantissa_multiplier32 u1 (.mant_a(mant_a), .mant_b(mant_b), .mant_result(mant_result));
  exponent_adder32 u2 (.exp_a(exp_a), .exp_b(exp_b), .exp_result(exp_result));
  sign_handler32 u3 (.sign_a(sign_a), .sign_b(sign_b), .sign_result(sign_result));
  normalizer32 u4 (.mant_result(mant_result), .exp_result(exp_result), .normalized_mant(normalized_mant), .normalized_exp(normalized_exp));

  assign Result = {sign_result, normalized_exp, normalized_mant};
  assign ALUFlags = (Result == 32'b0) ? 4'b0100 : 4'b0000; // Zero flag
endmodule




module mantissa_multiplier16 (
    input [10:0] mant_a,
    input [10:0] mant_b,
    output [21:0] mant_result
  );
  assign mant_result = mant_a * mant_b;
endmodule

module exponent_adder16 (
    input [4:0] exp_a,
    input [4:0] exp_b,
    output [4:0] exp_result
  );
  assign exp_result = exp_a + exp_b - 5'd15;
endmodule

module sign_handler16 (
    input sign_a,
    input sign_b,
    output sign_result
  );
  assign sign_result = sign_a ^ sign_b;
endmodule

module normalizer16 (
    input [21:0] mant_result,
    input [4:0] exp_result,
    output [9:0] normalized_mant,
    output [4:0] normalized_exp
  );
  wire [21:0] shifted_mant;
  wire [4:0] adjusted_exp;

  assign shifted_mant = mant_result >> 1;
  assign adjusted_exp = exp_result + 1;

  assign normalized_mant = mant_result[21] ? shifted_mant[20:11] : mant_result[19:10];
  assign normalized_exp = mant_result[21] ? adjusted_exp : exp_result;
endmodule



module fpmul16 (
    input [15:0] a,
    input [15:0] b,
    output [15:0] Result,
    output [3:0] ALUFlags
  );
  wire [4:0] exp_a, exp_b, exp_result;
  wire [10:0] mant_a, mant_b;
  wire [21:0] mant_result;
  wire sign_a, sign_b, sign_result;
  wire [9:0] normalized_mant;
  wire [4:0] normalized_exp;

  assign sign_a = a[15];
  assign sign_b = b[15];
  assign exp_a = a[14:10];
  assign exp_b = b[14:10];
  assign mant_a = {1'b1, a[9:0]};
  assign mant_b = {1'b1, b[9:0]};

  mantissa_multiplier16 u1 (.mant_a(mant_a), .mant_b(mant_b), .mant_result(mant_result));
  exponent_adder16 u2 (.exp_a(exp_a), .exp_b(exp_b), .exp_result(exp_result));
  sign_handler16 u3 (.sign_a(sign_a), .sign_b(sign_b), .sign_result(sign_result));
  normalizer16 u4 (.mant_result(mant_result), .exp_result(exp_result), .normalized_mant(normalized_mant), .normalized_exp(normalized_exp));

  assign Result = {sign_result, normalized_exp, normalized_mant};
  assign ALUFlags = (Result == 16'b0) ? 4'b0100 : 4'b0000; // Zero flag
endmodule
