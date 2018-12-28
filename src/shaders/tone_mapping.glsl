float tonemap_reinhard(float L, float one_over_LWhite_squared) {
    return L * (1 + L*one_over_LWhite_squared) / (1 + L);
}
