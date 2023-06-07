
volatile long left_enc_pos = 0L;
volatile long right_enc_pos = 0L;
static const int8_t ENC_STATES [] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};  //encoder lookup table
  
/* Interrupt routine for LEFT encoder, taking care of actual counting */
ISR (PCINT2_vect){
  static uint8_t enc_last=0;
      
enc_last <<=2; //shift previous state two places
enc_last |= (PIND & (3 << 2)) >> 2; //read the current state into lowest 2 bits

  left_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Interrupt routine for RIGHT encoder, taking care of actual counting */
ISR (PCINT1_vect){
      static uint8_t enc_last=0;
          
enc_last <<=2; //shift previous state two places
enc_last |= (PINC & (3 << 4)) >> 4; //read the current state into lowest 2 bits

  right_enc_pos += ENC_STATES[(enc_last & 0x0f)];
}

/* Wrap the encoder reading function */
long readEncoder(int i) {
  if (i == LEFT) return left_enc_pos;
  else return right_enc_pos;
}

/* Wrap the encoder reset function */
void resetEncoder(int i) {
  if (i == LEFT){
    left_enc_pos=0L;
    return;
  } else { 
    right_enc_pos=0L;
    return;
  }
}

/* Wrap the encoder reset function */
void resetEncoders() {
  resetEncoder(LEFT);
  resetEncoder(RIGHT);
}
